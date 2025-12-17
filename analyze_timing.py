#!/usr/bin/env python3
"""
Analyze timing results from AprilTag CUDA detector performance tests.
Extracts key metrics and formats them for easy comparison.
"""

import re
import sys
from pathlib import Path

def extract_timing_stats(content):
    """Extract timing statistics from test output."""
    stats = {}
    
    # Extract total pipeline stats
    pipeline_match = re.search(r'Total Pipeline Time \(per frame\):\s+Average:\s+([\d.]+)\s+ms\s+FPS:\s+([\d.]+)', content)
    if pipeline_match:
        stats['pipeline_avg_ms'] = float(pipeline_match.group(1))
        stats['pipeline_fps'] = float(pipeline_match.group(2))
    
    # Extract frame count and processing time
    frame_match = re.search(r'Completed processing (\d+) frames in ([\d.]+) seconds', content)
    if frame_match:
        stats['total_frames'] = int(frame_match.group(1))
        stats['total_time_sec'] = float(frame_match.group(2))
        stats['processing_fps'] = stats['total_frames'] / stats['total_time_sec']
    
    # Extract detection counts
    det_match = re.search(r'Total detections before filtering: (\d+).*?Total detections after filtering: (\d+)', content, re.DOTALL)
    if det_match:
        stats['detections_before'] = int(det_match.group(1))
        stats['detections_after'] = int(det_match.group(2))
        stats['avg_detections_before'] = stats['detections_before'] / stats['total_frames'] if stats['total_frames'] > 0 else 0
        stats['avg_detections_after'] = stats['detections_after'] / stats['total_frames'] if stats['total_frames'] > 0 else 0
    
    # Extract frame detection statistics (before and after filtering)
    before_section = re.search(r'BEFORE filtering:.*?Frames with tags detected: (\d+)\s+\(([\d.]+)%\).*?Frames without tags: (\d+)\s+\(([\d.]+)%\)', content, re.DOTALL)
    after_section = re.search(r'AFTER filtering:.*?Frames with tags detected: (\d+)\s+\(([\d.]+)%\).*?Frames without tags: (\d+)\s+\(([\d.]+)%\)', content, re.DOTALL)
    
    if before_section:
        stats['frames_with_detections_before'] = int(before_section.group(1))
        stats['frames_with_detections_before_pct'] = float(before_section.group(2))
        stats['frames_without_detections_before'] = int(before_section.group(3))
        stats['frames_without_detections_before_pct'] = float(before_section.group(4))
    
    if after_section:
        stats['frames_with_detections_after'] = int(after_section.group(1))
        stats['frames_with_detections_after_pct'] = float(after_section.group(2))
        stats['frames_without_detections_after'] = int(after_section.group(3))
        stats['frames_without_detections_after_pct'] = float(after_section.group(4))
    
    # Extract component timings
    components = [
        'Frame Read', 'Color Conversion', 'GPU Detection \(Total\)',
        'CUDA Operations \(GPU\)', 'CPU Decode \(Tag ID\)',
        'Coordinate Scaling', 'Filter Duplicates', 'Display \(imshow\)'
    ]
    
    for component in components:
        pattern = rf'{re.escape(component)}:.*?Average:\s+([\d.]+)\s+ms'
        match = re.search(pattern, content, re.DOTALL)
        if match:
            key = component.lower().replace(' ', '_').replace('(', '').replace(')', '').replace('_total', '').replace('tag_id_', '')
            stats[f'{key}_avg_ms'] = float(match.group(1))
    
    # Extract GPU detection breakdown
    gpu_match = re.search(r'GPU Detection \(Total\):.*?Average:\s+([\d.]+)\s+ms', content, re.DOTALL)
    cuda_match = re.search(r'CUDA Operations \(GPU\):.*?Average:\s+([\d.]+)\s+ms', content, re.DOTALL)
    decode_match = re.search(r'CPU Decode \(Tag ID\):.*?Average:\s+([\d.]+)\s+ms', content, re.DOTALL)
    
    if gpu_match and cuda_match and decode_match:
        stats['gpu_detect_total_ms'] = float(gpu_match.group(1))
        stats['cuda_ops_ms'] = float(cuda_match.group(1))
        stats['cpu_decode_ms'] = float(decode_match.group(1))
    
    # Extract filtering parameters
    filter_params = {}
    min_cluster_match = re.search(r'min_cluster_pixels:\s+(\d+)', content)
    if min_cluster_match:
        filter_params['min_cluster_pixels'] = int(min_cluster_match.group(1))
    
    max_line_mse_match = re.search(r'max_line_fit_mse:\s+([\d.]+)', content)
    if max_line_mse_match:
        filter_params['max_line_fit_mse'] = float(max_line_mse_match.group(1))
    
    min_diff_match = re.search(r'min_white_black_diff:\s+(\d+)', content)
    if min_diff_match:
        filter_params['min_white_black_diff'] = int(min_diff_match.group(1))
    
    if filter_params:
        stats['filter_params'] = filter_params
    
    return stats

def format_report(stats):
    """Format statistics into a readable report."""
    lines = []
    lines.append("=" * 70)
    lines.append("PERFORMANCE SUMMARY")
    lines.append("=" * 70)
    lines.append("")
    
    if 'total_frames' in stats:
        lines.append(f"Total Frames Processed: {stats['total_frames']}")
        lines.append(f"Total Time: {stats['total_time_sec']:.2f} seconds")
        lines.append(f"Processing FPS: {stats['processing_fps']:.2f}")
        lines.append("")
    
    if 'pipeline_avg_ms' in stats:
        lines.append("Pipeline Performance:")
        lines.append(f"  Average per frame: {stats['pipeline_avg_ms']:.3f} ms")
        lines.append(f"  Theoretical FPS: {stats['pipeline_fps']:.2f}")
        lines.append("")
    
    if 'detections_before' in stats:
        lines.append("Detection Statistics:")
        lines.append(f"  Before filtering: {stats['detections_before']} ({stats['avg_detections_before']:.2f}/frame)")
        lines.append(f"  After filtering: {stats['detections_after']} ({stats['avg_detections_after']:.2f}/frame)")
        lines.append(f"  Filtered: {stats['detections_before'] - stats['detections_after']} ({100 * (1 - stats['detections_after']/stats['detections_before']):.1f}%)")
        lines.append("")
    
    if 'frames_with_detections_before' in stats:
        lines.append("Frame Detection Coverage:")
        lines.append("  BEFORE filtering:")
        lines.append(f"    Frames with tags detected: {stats['frames_with_detections_before']} ({stats['frames_with_detections_before_pct']:.1f}%)")
        lines.append(f"    Frames without tags: {stats['frames_without_detections_before']} ({stats['frames_without_detections_before_pct']:.1f}%)")
        if stats['frames_with_detections_before'] > 0:
            avg_detections_per_detected_frame = stats['detections_before'] / stats['frames_with_detections_before']
            lines.append(f"    Avg detections per detected frame: {avg_detections_per_detected_frame:.2f}")
        lines.append("  AFTER filtering:")
        if 'frames_with_detections_after' in stats:
            lines.append(f"    Frames with tags detected: {stats['frames_with_detections_after']} ({stats['frames_with_detections_after_pct']:.1f}%)")
            lines.append(f"    Frames without tags: {stats['frames_without_detections_after']} ({stats['frames_without_detections_after_pct']:.1f}%)")
            if stats['frames_with_detections_after'] > 0:
                avg_detections_per_detected_frame = stats['detections_after'] / stats['frames_with_detections_after']
                lines.append(f"    Avg detections per detected frame: {avg_detections_per_detected_frame:.2f}")
            # Show filtering impact
            frames_lost = stats['frames_with_detections_before'] - stats['frames_with_detections_after']
            if frames_lost > 0:
                lines.append(f"    Frames lost due to filtering: {frames_lost} ({100.0 * frames_lost / stats['frames_with_detections_before']:.1f}%)")
        lines.append("")
    
    if 'gpu_detect_total_ms' in stats:
        lines.append("Component Timing Breakdown (ms/frame):")
        lines.append(f"  GPU Detection (Total):     {stats['gpu_detect_total_ms']:>8.3f} ms ({100*stats['gpu_detect_total_ms']/stats['pipeline_avg_ms']:>5.1f}%)")
        lines.append(f"    ├─ CUDA Operations:      {stats['cuda_ops_ms']:>8.3f} ms ({100*stats['cuda_ops_ms']/stats['pipeline_avg_ms']:>5.1f}%)")
        lines.append(f"    └─ CPU Decode:           {stats['cpu_decode_ms']:>8.3f} ms ({100*stats['cpu_decode_ms']/stats['pipeline_avg_ms']:>5.1f}%)")
        lines.append("")
    
    # Add other components
    other_components = [
        ('frame_read_avg_ms', 'Frame Read'),
        ('color_conversion_avg_ms', 'Color Conversion'),
        ('coordinate_scaling_avg_ms', 'Coordinate Scaling'),
        ('filter_duplicates_avg_ms', 'Filter Duplicates'),
        ('display_imshow_avg_ms', 'Display (imshow)')
    ]
    
    other_found = False
    for key, name in other_components:
        if key in stats and stats[key] > 0:
            if not other_found:
                lines.append("Other Components:")
                other_found = True
            pct = 100 * stats[key] / stats['pipeline_avg_ms'] if 'pipeline_avg_ms' in stats else 0
            lines.append(f"  {name:25} {stats[key]:>8.3f} ms ({pct:>5.1f}%)")
    
    if other_found:
        lines.append("")
    
    if 'filter_params' in stats:
        lines.append("Filtering Parameters:")
        for key, value in stats['filter_params'].items():
            lines.append(f"  {key}: {value}")
        lines.append("")
    
    lines.append("=" * 70)
    
    return "\n".join(lines)

def main():
    if len(sys.argv) < 2:
        print("Usage: ./analyze_timing.py <test_output_file>")
        sys.exit(1)
    
    input_file = Path(sys.argv[1])
    if not input_file.exists():
        print(f"Error: File not found: {input_file}")
        sys.exit(1)
    
    content = input_file.read_text()
    stats = extract_timing_stats(content)
    
    report = format_report(stats)
    print(report)
    
    # Save formatted report in test_results directory
    # If input file is already in test_results, save there; otherwise create test_results
    if "test_results" in str(input_file.parent):
        output_file = input_file.parent / f"{input_file.stem}_summary.txt"
    else:
        test_results_dir = input_file.parent / "test_results"
        test_results_dir.mkdir(exist_ok=True)
        output_file = test_results_dir / f"{input_file.stem}_summary.txt"
    output_file.write_text(report)
    print(f"\nFormatted report saved to: {output_file}")

if __name__ == '__main__':
    main()

