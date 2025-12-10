#!/bin/bash
# Advanced conversion script with custom template

INPUT_FILE="MAPPING_MODULE_TECHNICAL_REPORT.md"
TEMPLATE="thesis_template.tex"
OUTPUT_TEX="MAPPING_MODULE_TECHNICAL_REPORT.tex"
OUTPUT_PDF="MAPPING_MODULE_TECHNICAL_REPORT.pdf"

echo "==========================================="
echo "Converting Markdown to LaTeX with Template"
echo "==========================================="
echo ""

# Check if pandoc is installed
if ! command -v pandoc &> /dev/null; then
    echo "❌ Pandoc is not installed!"
    echo ""
    echo "Install with:"
    echo "  sudo apt-get install pandoc"
    echo "  or"
    echo "  sudo snap install pandoc"
    exit 1
fi

# Convert with template
echo "Step 1: Converting MD → LaTeX..."
pandoc "$INPUT_FILE" \
    -f markdown+tex_math_dollars \
    -t latex \
    --template="$TEMPLATE" \
    --toc \
    --number-sections \
    --highlight-style=tango \
    -o "$OUTPUT_TEX"

if [ $? -eq 0 ]; then
    echo "✓ LaTeX file created: $OUTPUT_TEX"
else
    echo "❌ Conversion failed!"
    exit 1
fi

echo ""
echo "Step 2: Compiling LaTeX → PDF..."

# Check if pdflatex is installed
if ! command -v pdflatex &> /dev/null; then
    echo "⚠ pdflatex not found. Install with:"
    echo "  sudo apt-get install texlive-latex-extra texlive-fonts-recommended"
    echo ""
    echo "You can still use the .tex file: $OUTPUT_TEX"
    exit 0
fi

# Compile to PDF (run twice for references)
pdflatex -interaction=nonstopmode "$OUTPUT_TEX" > /dev/null 2>&1
pdflatex -interaction=nonstopmode "$OUTPUT_TEX" > /dev/null 2>&1

if [ -f "$OUTPUT_PDF" ]; then
    echo "✓ PDF created: $OUTPUT_PDF"
    echo ""
    echo "==========================================="
    echo "✅ Conversion Complete!"
    echo "==========================================="
    echo ""
    echo "Output files:"
    echo "  LaTeX: $OUTPUT_TEX"
    echo "  PDF:   $OUTPUT_PDF"
    echo ""
    echo "To view PDF:"
    echo "  evince $OUTPUT_PDF &"
else
    echo "⚠ PDF compilation had warnings. Check $OUTPUT_TEX manually."
    echo ""
    echo "Compile manually with:"
    echo "  pdflatex $OUTPUT_TEX"
fi

# Cleanup auxiliary files
rm -f *.aux *.log *.out *.toc

echo ""
echo "Done!"
