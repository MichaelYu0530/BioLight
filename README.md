# Gel Electrophoresis Band Detection Algorithms

**Author**: Yue Yu | **USTC Statistics 2025**

## Overview
Three algorithm versions for protein band detection in gel electrophoresis images.

## Three Algorithm Versions

### PlanA: Initial Exploratory Implementation
- First working version of "band-first" paradigm

### PlanB: Optimized Version  
- Direct "image → band" detection path
- Geometric modeling for tilted bands

### PlanC: Production-Ready Version
- Simplified architecture
- **30% accuracy improvement** over traditional methods
- Deployed in production at Biolight Biotech

## Build Instructions

```bash
mkdir build && cd build
cmake .. && make
./planA_demo   # or ./planB_demo, ./planC_demo
```

## License

MIT License