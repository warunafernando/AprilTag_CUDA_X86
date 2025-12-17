#!/usr/bin/env python3
"""
Detailed performance analysis tool to identify bottlenecks and optimization opportunities.
"""

import sys
import re
from pathlib import Path
from analyze_timing import extract_timing_stats

def analyze_bottlenecks(stats):
    """Analyze timing statistics to identify bottlenecks."""
    if 'pipeline_avg_ms' not in stats:
        print("DEBUG: pipeline_avg_ms not found in stats", file=sys.stderr)
        print("DEBUG: Available keys:", list(stats.keys())[:20], file=sys.stderr)
        return []
    
    pipeline_ms = stats['pipeline_avg_ms']
    bottlenecks = []
    
    # Analyze each component
    components = [
        ('gpu_detect_total_ms', 'GPU Detection (Total)', 'critical'),
        ('cuda_ops_ms', 'CUDA Operations', 'critical'),
        ('cpu_decode_ms', 'CPU Decode', 'critical'),
        ('frame_read_avg_ms', 'Frame Read', 'medium'),
        ('color_conversion_avg_ms', 'Color Conversion', 'low'),
        ('display_imshow_avg_ms', 'Display (imshow)', 'medium'),
        ('coordinate_scaling_avg_ms', 'Coordinate Scaling', 'low'),
        ('filter_duplicates_avg_ms', 'Filter Duplicates', 'low'),
    ]
    
    for key, name, priority in components:
        if key in stats and stats[key] > 0:
            time_ms = stats[key]
            percentage = 100 * time_ms / pipeline_ms if pipeline_ms > 0 else 0
            bottlenecks.append({
                'name': name,
                'time_ms': time_ms,
                'percentage': percentage,
                'priority': priority
            })
    
    # Sort by time (largest first)
    bottlenecks.sort(key=lambda x: x['time_ms'], reverse=True)
    return bottlenecks

def generate_recommendations(stats, bottlenecks):
    """Generate optimization recommendations based on analysis."""
    recommendations = []
    
    if not bottlenecks:
        return recommendations
    
    pipeline_ms = stats.get('pipeline_avg_ms', 0)
    total_gpu_ms = stats.get('gpu_detect_total_ms', 0)
    cuda_ms = stats.get('cuda_ops_ms', 0)
    cpu_decode_ms = stats.get('cpu_decode_ms', 0)
    
    # Analyze GPU Detection breakdown
    if total_gpu_ms > 0:
        cuda_percent = 100 * cuda_ms / total_gpu_ms if total_gpu_ms > 0 else 0
        decode_percent = 100 * cpu_decode_ms / total_gpu_ms if total_gpu_ms > 0 else 0
        
        # CPU decode is significant if it's > 0.3ms or > 10% of pipeline
        if cpu_decode_ms > 0.3 or (cpu_decode_ms / pipeline_ms > 0.1):
            recommendations.append({
                'priority': 'HIGH',
                'area': 'CPU Decode',
                'current_time': f'{cpu_decode_ms:.3f} ms',
                'percentage': f'{100 * cpu_decode_ms / pipeline_ms:.1f}%',
                'recommendation': 'Port CPU decode operations to GPU (estimated 2-3 weeks)',
                'potential_savings': f'~{cpu_decode_ms * 0.8:.3f} ms ({100 * cpu_decode_ms * 0.8 / pipeline_ms:.1f}% of pipeline)',
                'effort': 'High (2-3 weeks)',
                'impact': f'Could reduce pipeline time by {100 * cpu_decode_ms * 0.8 / pipeline_ms:.1f}% (80% of decode time)'
            })
    
    # Analyze CUDA operations
    if cuda_ms > 1.0:  # More than 1ms (significant component)
        recommendations.append({
            'priority': 'MEDIUM',
            'area': 'CUDA Operations',
            'current_time': f'{cuda_ms:.3f} ms',
            'percentage': f'{100 * cuda_ms / pipeline_ms:.1f}%',
            'recommendation': 'Consider increasing quad_decimate (2.0 → 2.5 or 3.0) in config.txt',
            'potential_savings': f'~{cuda_ms * 0.15:.3f} ms (estimated 15% reduction)',
            'effort': 'Very Low (config change)',
            'impact': f'Could reduce CUDA time by ~15%. Trade-off: Slightly reduced detection accuracy for distant tags'
        })
    
    # Display overhead
    display_ms = stats.get('display_imshow_avg_ms', 0)
    if display_ms > 0.5:
        recommendations.append({
            'priority': 'MEDIUM',
            'area': 'Display (imshow)',
            'current_time': f'{display_ms:.3f} ms',
            'percentage': f'{100 * display_ms / pipeline_ms:.1f}%',
            'recommendation': 'Remove display for production/headless use',
            'potential_savings': f'{display_ms:.3f} ms ({100 * display_ms / pipeline_ms:.1f}% of pipeline)',
            'effort': 'Very Low (code change)',
            'impact': 'Immediate ~{:.1f}% improvement'.format(100 * display_ms / pipeline_ms)
        })
    
    # Frame read optimization
    frame_read_ms = stats.get('frame_read_avg_ms', 0)
    if frame_read_ms > 0.7:
        recommendations.append({
            'priority': 'MEDIUM',
            'area': 'Frame Read',
            'current_time': f'{frame_read_ms:.3f} ms',
            'percentage': f'{100 * frame_read_ms / pipeline_ms:.1f}%',
            'recommendation': 'Implement frame prefetching in background thread',
            'potential_savings': f'~{frame_read_ms * 0.4:.3f} ms (estimated 40% reduction)',
            'effort': 'Medium (threading)',
            'impact': f'Could reduce pipeline time by {100 * frame_read_ms * 0.4 / pipeline_ms:.1f}%'
        })
    
    # Frame read optimization (already included above, but ensure it's there)
    # Check if filtering is efficient
    filter_ms = stats.get('filter_duplicates_avg_ms', 0)
    if filter_ms < 0.01 and len([r for r in recommendations if 'Filtering' in r['area']]) == 0:
        recommendations.append({
            'priority': 'LOW',
            'area': 'Filtering',
            'current_time': f'{filter_ms:.3f} ms',
            'percentage': f'{100 * filter_ms / pipeline_ms:.1f}%',
            'recommendation': 'Filtering is already highly optimized',
            'potential_savings': 'Negligible',
            'effort': 'N/A',
            'impact': 'No improvement needed'
        })
    
    return recommendations

def format_analysis(stats, bottlenecks, recommendations):
    """Format the complete analysis report."""
    lines = []
    lines.append("=" * 80)
    lines.append("DETAILED PERFORMANCE ANALYSIS")
    lines.append("=" * 80)
    lines.append("")
    
    # Overall performance
    if 'pipeline_avg_ms' in stats:
        lines.append("Overall Performance:")
        lines.append(f"  Pipeline Time: {stats['pipeline_avg_ms']:.3f} ms per frame")
        lines.append(f"  Theoretical FPS: {stats.get('pipeline_fps', 0):.2f}")
        if 'processing_fps' in stats:
            lines.append(f"  Actual Processing FPS: {stats['processing_fps']:.2f}")
        lines.append("")
    
    # Bottleneck analysis
    lines.append("Bottleneck Analysis (sorted by time):")
    lines.append("")
    for i, bottleneck in enumerate(bottlenecks, 1):
        priority_symbol = {'critical': '🔴', 'medium': '🟡', 'low': '🟢'}.get(bottleneck['priority'], '⚪')
        lines.append(f"  {i}. {priority_symbol} {bottleneck['name']:30} "
                    f"{bottleneck['time_ms']:>8.3f} ms ({bottleneck['percentage']:>5.1f}%)")
    lines.append("")
    
    # GPU Detection breakdown
    if 'gpu_detect_total_ms' in stats:
        lines.append("GPU Detection Breakdown:")
        total = stats['gpu_detect_total_ms']
        cuda = stats.get('cuda_ops_ms', 0)
        decode = stats.get('cpu_decode_ms', 0)
        lines.append(f"  Total: {total:.3f} ms")
        lines.append(f"    ├─ CUDA Operations: {cuda:.3f} ms ({100 * cuda / total:.1f}%)")
        lines.append(f"    └─ CPU Decode:      {decode:.3f} ms ({100 * decode / total:.1f}%)")
        lines.append("")
    
    # Optimization recommendations
    if recommendations:
        lines.append("Optimization Recommendations:")
        lines.append("")
        for i, rec in enumerate(recommendations, 1):
            lines.append(f"  {i}. [{rec['priority']}] {rec['area']}")
            lines.append(f"     Current: {rec['current_time']} ({rec['percentage']} of pipeline)")
            lines.append(f"     Recommendation: {rec['recommendation']}")
            lines.append(f"     Potential Savings: {rec['potential_savings']}")
            lines.append(f"     Effort: {rec['effort']}")
            lines.append(f"     Impact: {rec['impact']}")
            lines.append("")
    
    # Calculate potential improvements
    total_savings = 0
    for rec in recommendations:
        if rec['potential_savings'] != 'Negligible' and 'ms' in rec['potential_savings']:
            try:
                savings_str = rec['potential_savings'].split()[0].replace('~', '')
                savings = float(savings_str)
                total_savings += savings
            except:
                pass
    
    if total_savings > 0 and 'pipeline_avg_ms' in stats:
        current_ms = stats['pipeline_avg_ms']
        potential_ms = current_ms - total_savings
        potential_fps = 1000.0 / potential_ms if potential_ms > 0 else 0
        improvement_pct = 100 * total_savings / current_ms
        
        lines.append("Potential Performance Improvements:")
        lines.append(f"  Current: {current_ms:.3f} ms ({stats.get('pipeline_fps', 0):.2f} FPS)")
        lines.append(f"  Potential: {potential_ms:.3f} ms ({potential_fps:.2f} FPS)")
        lines.append(f"  Improvement: {total_savings:.3f} ms ({improvement_pct:.1f}% faster)")
        lines.append("")
    
    lines.append("=" * 80)
    
    return "\n".join(lines)

def main():
    if len(sys.argv) < 2:
        print("Usage: ./analyze_performance.py <test_output_file>")
        sys.exit(1)
    
    input_file = Path(sys.argv[1])
    if not input_file.exists():
        print(f"Error: File not found: {input_file}")
        sys.exit(1)
    
    content = input_file.read_text()
    stats = extract_timing_stats(content)
    
    bottlenecks = analyze_bottlenecks(stats)
    recommendations = generate_recommendations(stats, bottlenecks)
    analysis = format_analysis(stats, bottlenecks, recommendations)
    
    print(analysis)
    
    # Save analysis report
    test_results_dir = input_file.parent if "test_results" in str(input_file.parent) else input_file.parent / "test_results"
    test_results_dir.mkdir(exist_ok=True)
    output_file = test_results_dir / f"{input_file.stem}_performance_analysis.txt"
    output_file.write_text(analysis)
    print(f"\nPerformance analysis saved to: {output_file}")

if __name__ == '__main__':
    main()

