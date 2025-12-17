# Quad Decimate Limitation

## Issue

The CUDA implementation has a **hardcoded check** that requires `quad_decimate` to be exactly **2.0**.

**Error when trying 2.5**:
```
Check failed: tag_detector_->quad_decimate == 2 (2.5 vs. 2)
```

## Location

`src/apriltags_cuda/src/apriltag_gpu.cu:173`

## Explanation

The CUDA kernel is optimized and compiled for a specific `quad_decimate` value (2.0). Changing this would require:
1. Modifying the CUDA kernel code
2. Recompiling the CUDA code
3. Potentially adjusting memory allocations and kernel parameters

## Current Status

- `quad_decimate` is **locked to 2.0** in the CUDA implementation
- This is a compile-time constant, not a runtime parameter
- Cannot be changed via config.txt without code modifications

## Alternative Optimizations

Since `quad_decimate` cannot be changed easily, focus on other optimizations:
1. ✅ **Frame prefetching** - Can still improve frame read time (17.5% of pipeline)
2. ✅ **GPU CPU decode port** - Can still improve CPU decode time (16.1% of pipeline)
3. ✅ **Display removal** - Can remove display overhead if not needed

## Conclusion

The `quad_decimate` optimization is **not feasible** without modifying and recompiling the CUDA kernel code. The current value of 2.0 is optimal for the compiled kernel.

