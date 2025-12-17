#!/usr/bin/env python3
"""
Compare two performance test results side-by-side.
Usage: ./compare_results.py <baseline_file> <new_file>
"""

import sys
import re
from pathlib import Path
from analyze_timing import extract_timing_stats

def format_comparison(baseline_stats, new_stats):
    """Format a side-by-side comparison of two test results."""
    lines = []
    lines.append("=" * 80)
    lines.append("PERFORMANCE COMPARISON")
    lines.append("=" * 80)
    lines.append("")
    
    # Basic info
    if 'total_frames' in baseline_stats and 'total_frames' in new_stats:
        lines.append(f"Frames Processed: {baseline_stats['total_frames']} (both)")
        lines.append("")
    
    # Pipeline performance
    if 'pipeline_avg_ms' in baseline_stats and 'pipeline_avg_ms' in new_stats:
        baseline_ms = baseline_stats['pipeline_avg_ms']
        new_ms = new_stats['pipeline_avg_ms']
        baseline_fps = baseline_stats.get('pipeline_fps', 1000.0 / baseline_ms)
        new_fps = new_stats.get('pipeline_fps', 1000.0 / new_ms)
        
        improvement_ms = baseline_ms - new_ms
        improvement_pct = 100 * improvement_ms / baseline_ms
        improvement_fps = new_fps - baseline_fps
        improvement_fps_pct = 100 * improvement_fps / baseline_fps
        
        lines.append("Pipeline Performance:")
        lines.append(f"  Baseline: {baseline_ms:>8.3f} ms ({baseline_fps:>6.2f} FPS)")
        lines.append(f"  New:      {new_ms:>8.3f} ms ({new_fps:>6.2f} FPS)")
        lines.append(f"  Change:   {improvement_ms:>+8.3f} ms ({improvement_pct:>+6.1f}%) | {improvement_fps:>+6.2f} FPS ({improvement_fps_pct:>+5.1f}%)")
        lines.append("")
    
    # GPU Detection breakdown
    if 'gpu_detect_total_ms' in baseline_stats and 'gpu_detect_total_ms' in new_stats:
        lines.append("GPU Detection Breakdown:")
        
        components = [
            ('gpu_detect_total_ms', 'GPU Detection (Total)'),
            ('cuda_ops_ms', '  ├─ CUDA Operations'),
            ('cpu_decode_ms', '  └─ CPU Decode')
        ]
        
        for key, label in components:
            if key in baseline_stats and key in new_stats:
                baseline_val = baseline_stats[key]
                new_val = new_stats[key]
                improvement = baseline_val - new_val
                improvement_pct = 100 * improvement / baseline_val if baseline_val > 0 else 0
                
                lines.append(f"{label:25} {baseline_val:>6.3f} → {new_val:>6.3f} ms ({improvement:>+6.3f}, {improvement_pct:>+5.1f}%)")
        lines.append("")
    
    # Other components
    other_components = [
        ('frame_read_avg_ms', 'Frame Read'),
        ('color_conversion_avg_ms', 'Color Conversion'),
        ('display_imshow_avg_ms', 'Display (imshow)')
    ]
    
    other_found = False
    for key, name in other_components:
        if key in baseline_stats and key in new_stats:
            if not other_found:
                lines.append("Other Components:")
                other_found = True
            baseline_val = baseline_stats[key]
            new_val = new_stats[key]
            improvement = baseline_val - new_val
            improvement_pct = 100 * improvement / baseline_val if baseline_val > 0 else 0
            
            lines.append(f"  {name:25} {baseline_val:>6.3f} → {new_val:>6.3f} ms ({improvement:>+6.3f}, {improvement_pct:>+5.1f}%)")
    
    if other_found:
        lines.append("")
    
    # Detection statistics
    if 'detections_before' in baseline_stats and 'detections_before' in new_stats:
        lines.append("Detection Statistics:")
        lines.append(f"  Before filtering: {baseline_stats['detections_before']} → {new_stats['detections_before']}")
        lines.append(f"  After filtering:  {baseline_stats['detections_after']} → {new_stats['detections_after']}")
        
        baseline_filter_pct = 100 * (1 - baseline_stats['detections_after'] / baseline_stats['detections_before']) if baseline_stats['detections_before'] > 0 else 0
        new_filter_pct = 100 * (1 - new_stats['detections_after'] / new_stats['detections_before']) if new_stats['detections_before'] > 0 else 0
        
        lines.append(f"  Filter rate:      {baseline_filter_pct:.1f}% → {new_filter_pct:.1f}%")
        lines.append("")
    
    # Frame detection coverage
    if 'frames_with_detections_before' in baseline_stats and 'frames_with_detections_before' in new_stats:
        lines.append("Frame Detection Coverage:")
        
        # BEFORE filtering
        lines.append("  BEFORE filtering:")
        baseline_with_before = baseline_stats['frames_with_detections_before']
        new_with_before = new_stats['frames_with_detections_before']
        baseline_with_before_pct = baseline_stats.get('frames_with_detections_before_pct', 0)
        new_with_before_pct = new_stats.get('frames_with_detections_before_pct', 0)
        
        lines.append(f"    Frames with tags:    {baseline_with_before} ({baseline_with_before_pct:.1f}%) → {new_with_before} ({new_with_before_pct:.1f}%)")
        
        baseline_without_before = baseline_stats['frames_without_detections_before']
        new_without_before = new_stats['frames_without_detections_before']
        baseline_without_before_pct = baseline_stats.get('frames_without_detections_before_pct', 0)
        new_without_before_pct = new_stats.get('frames_without_detections_before_pct', 0)
        
        lines.append(f"    Frames without tags: {baseline_without_before} ({baseline_without_before_pct:.1f}%) → {new_without_before} ({new_without_before_pct:.1f}%)")
        
        if baseline_with_before > 0 and new_with_before > 0:
            baseline_avg_per_frame = baseline_stats['detections_before'] / baseline_with_before
            new_avg_per_frame = new_stats['detections_before'] / new_with_before
            lines.append(f"    Avg per detected frame: {baseline_avg_per_frame:.2f} → {new_avg_per_frame:.2f}")
        
        # AFTER filtering
        if 'frames_with_detections_after' in baseline_stats and 'frames_with_detections_after' in new_stats:
            lines.append("  AFTER filtering:")
            baseline_with_after = baseline_stats['frames_with_detections_after']
            new_with_after = new_stats['frames_with_detections_after']
            baseline_with_after_pct = baseline_stats.get('frames_with_detections_after_pct', 0)
            new_with_after_pct = new_stats.get('frames_with_detections_after_pct', 0)
            
            lines.append(f"    Frames with tags:    {baseline_with_after} ({baseline_with_after_pct:.1f}%) → {new_with_after} ({new_with_after_pct:.1f}%)")
            
            baseline_without_after = baseline_stats['frames_without_detections_after']
            new_without_after = new_stats['frames_without_detections_after']
            baseline_without_after_pct = baseline_stats.get('frames_without_detections_after_pct', 0)
            new_without_after_pct = new_stats.get('frames_without_detections_after_pct', 0)
            
            lines.append(f"    Frames without tags: {baseline_without_after} ({baseline_without_after_pct:.1f}%) → {new_without_after} ({new_without_after_pct:.1f}%)")
            
            if baseline_with_after > 0 and new_with_after > 0:
                baseline_avg_per_frame = baseline_stats['detections_after'] / baseline_with_after
                new_avg_per_frame = new_stats['detections_after'] / new_with_after
                lines.append(f"    Avg per detected frame: {baseline_avg_per_frame:.2f} → {new_avg_per_frame:.2f}")
        
        lines.append("")
    
    # Filtering parameters
    if 'filter_params' in baseline_stats or 'filter_params' in new_stats:
        lines.append("Filtering Parameters:")
        params = set()
        if 'filter_params' in baseline_stats:
            params.update(baseline_stats['filter_params'].keys())
        if 'filter_params' in new_stats:
            params.update(new_stats['filter_params'].keys())
        
        for param in sorted(params):
            baseline_val = baseline_stats.get('filter_params', {}).get(param, 'N/A')
            new_val = new_stats.get('filter_params', {}).get(param, 'N/A')
            lines.append(f"  {param:25} {baseline_val} → {new_val}")
        lines.append("")
    
    lines.append("=" * 80)
    
    return "\n".join(lines)

def main():
    if len(sys.argv) < 3:
        print("Usage: ./compare_results.py <baseline_file> <new_file>")
        sys.exit(1)
    
    baseline_file = Path(sys.argv[1])
    new_file = Path(sys.argv[2])
    
    if not baseline_file.exists():
        print(f"Error: Baseline file not found: {baseline_file}")
        sys.exit(1)
    
    if not new_file.exists():
        print(f"Error: New file not found: {new_file}")
        sys.exit(1)
    
    baseline_content = baseline_file.read_text()
    new_content = new_file.read_text()
    
    baseline_stats = extract_timing_stats(baseline_content)
    new_stats = extract_timing_stats(new_content)
    
    comparison = format_comparison(baseline_stats, new_stats)
    print(comparison)
    
    # Save comparison in test_results directory
    test_results_dir = Path("test_results")
    test_results_dir.mkdir(exist_ok=True)
    output_file = test_results_dir / f"comparison_{baseline_file.stem}_vs_{new_file.stem}.txt"
    output_file.write_text(comparison)
    print(f"\nComparison saved to: {output_file}")

if __name__ == '__main__':
    main()

