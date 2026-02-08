# Gel Electrophoresis Analysis: Three-Phase Research Journey

**Author**: Yue Yu | **USTC Statistics 2025**  
**Period**: 2022-2023 Internships at BioLight Biotech

## Overview
This repository documents my complete three-phase research journey in developing gel electrophoresis image analysis algorithms, demonstrating technical evolution from algorithmic innovation to deep learning exploration.

## Demonstration

Check out the Python demonstration:

```bash
cd demo
pip install -r requirements.txt  # Only needs numpy and opencv-python
python visualization.py
```

The demo generates 8 visualizations showing:

- Single band detection with guide lines
- Multi-lane "band-first-then-lane" workflow
- Phase 1 vs Phase 2 detection comparison
- All results saved to `demo/images/`

## Research Phases

### [Phase 1: Algorithm Innovation](/phase1_band_first_algorithm)
**July-August 2022** | **Complete C++ Implementation Available**
- **Problem**: Traditional algorithms fail completely on skewed lanes
- **Innovation**: "Band-first-then-lane" paradigm shift
- **Outcome**: **30% accuracy improvement**, production deployment

### [Phase 2: Algorithm Optimization](/phase2_algorithm_optimization)  
**January-February 2023** | **Methodology & Performance Analysis**
- **Problem**: Low-contrast bands, rigid output format
- **Solutions**: Global contrast enhancement, contour-aware detection
- **Outcome**: **Additional 10% accuracy gain** on challenging cases

### [Phase 3: Deep Learning Exploration](/phase3_deep_learning_pilot)
**August 2023** | **Experimental Research & Comparison**
- **Research Question**: Can DL match hand-crafted algorithm performance?
- **Approach**: Faster R-CNN with domain-specific adaptations
- **Finding**: **Accuracy parity with 50-70% GPU speed advantage**

## Technical Highlights

### Core Innovations
1. **Paradigm Shift**: "Band-first-then-lane" vs traditional "lane-first-then-band"
2. **Targeted Optimization**: Contrast enhancement for low-visibility bands and contour detection for tooth-like bands
3. **Multi-Approach Validation**: Traditional CV and deep learning comparison

### Performance Impact
- **Overall**: 40% cumulative accuracy improvement over legacy systems
- **Edge Cases**: Resolved complete failure on skewed lanes
- **Production**: Deployed in company's analysis software

### Evaluation Methodology & Results

**Test Set Composition**:

- **Total**: 40 gel electrophoresis images
- **Ideal Cases (20)**: Straight lanes, rectangular bands, good contrast
- **Challenging Cases (20)**: Skewed lanes, irregular bands, low contrast, noise

**Accuracy Calculation**:

1. Algorithms process all 40 images
2. Each result is manually verified
3. **Success criteria**: All bands detected, zero false positives
4. **Accuracy** = (Correctly identified images) / 40 × 100%

**Phase-wise Results**:

- **Legacy**: ~ 50% accuracy(high in ideal cases but low in challenging ones)
- **Phase 1**: ~ 80% accuracy (nearly 100% in ideal cases and significant improvement in challenging ones)
- **Phase 2**: ~ 90% accuracy (further improvement in challenging cases)

## Repository Structure

BioLight/
├── phase1_band_first_algorithm/  # Phase 1: C++ algorithm design
│   ├── src/                      # Source code architecture
│   ├── CMakeLists.txt            # Build configuration
│   └── README.md                 # Technical documentation
├── phase2_algorithm_optimization/ # Phase 2: Optimization methodology
│   └── README.md                 # Technical solutions & results
├── phase3_deep_learning_pilot/   # Phase 3: ML exploration
│   └── README.md                 # Experimental design & findings
├── demo/                         # Python Visualization
│   ├── visualization.py     # Main demonstration script
│   ├── gel_functions.py          # Core algorithm implementation
│   ├── requirements.txt          # Only numpy + opencv-python
│   ├── README.md                 # Demo-specific instructions
│   └── images/                   # 8 generated result images
└── README.md                     # Synthesis & academic relevance

**Code Availability**: 
Phase 2-3 implementations are company property; this repository focuses on research documentation and a Python demonstration of the core algorithms.

## Quick Start

### Explore Phase 1 (Code Implementation)
```bash
# Clone repository
git clone https://github.com/MichaelYu0530/BioLight.git
cd BioLight/phase1_band_first_algorithm
```

## Research Significance

This work demonstrates:

- **Problem-solving**: Identifying fundamental flaws in existing approaches
- **Technical evolution**: From concept to production to research exploration
- **Methodological rigor**: Quantitative validation and comparative analysis
- **Interdisciplinary synthesis**: Statistics, computer vision, machine learning

## Author

**Yue Yu**
University of Science and Technology of China (USTC)
Bachelor of Science in Statistics, Expected 2025

**Contact**: yueyu0530@gmail.com or 3344792547@qq.com
**GitHub**: [MichaelYu0530](https://github.com/MichaelYu0530)

## License

This repository is licensed under the MIT License - see the [LICENSE](https://license/) file for details.

## Acknowledgments

- Mentors at **Guangzhou Biolight Biotechnology** for guidance and domain expertise
- **USTC faculty** for statistical and mathematical foundation
- **OpenCV and TensorFlow communities** for excellent open-source libraries