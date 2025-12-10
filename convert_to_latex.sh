#!/bin/bash
# Script to convert Markdown technical report to LaTeX

INPUT_FILE="MAPPING_MODULE_TECHNICAL_REPORT.md"
OUTPUT_FILE="MAPPING_MODULE_TECHNICAL_REPORT.tex"

echo "Converting $INPUT_FILE to LaTeX..."

pandoc "$INPUT_FILE" \
    -f markdown \
    -t latex \
    --standalone \
    --toc \
    --number-sections \
    --highlight-style=tango \
    --variable geometry:margin=1in \
    --variable fontsize=11pt \
    --variable documentclass=article \
    --variable classoption=twocolumn \
    -o "$OUTPUT_FILE"

echo "✓ Conversion complete: $OUTPUT_FILE"
echo ""
echo "To compile to PDF:"
echo "  pdflatex $OUTPUT_FILE"
echo "  pdflatex $OUTPUT_FILE  # Run twice for references"
