#!/bin/bash
# extract_books.sh - Extract text and images from Car Wars PDF books
# Run this if the books/images or books/*.txt files are lost
# Requires: pdftotext, pdfimages (from poppler-utils)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BOOKS_DIR="$PROJECT_ROOT/books"

# Check dependencies
check_deps() {
    local missing=()
    command -v pdftotext >/dev/null 2>&1 || missing+=("pdftotext")
    command -v pdfimages >/dev/null 2>&1 || missing+=("pdfimages")

    if [ ${#missing[@]} -gt 0 ]; then
        echo "ERROR: Missing required tools: ${missing[*]}"
        echo "Install with: sudo apt install poppler-utils"
        exit 1
    fi
}

# Extract text from PDF
extract_text() {
    local pdf="$1"
    local txt="$2"

    if [ -f "$pdf" ]; then
        echo "Extracting text: $(basename "$pdf") -> $(basename "$txt")"
        pdftotext -layout "$pdf" "$txt"
        echo "  Lines: $(wc -l < "$txt")"
    else
        echo "WARNING: PDF not found: $pdf"
    fi
}

# Extract images from PDF
extract_images() {
    local pdf="$1"
    local output_dir="$2"

    if [ -f "$pdf" ]; then
        mkdir -p "$output_dir"
        echo "Extracting images: $(basename "$pdf") -> $output_dir/"
        pdfimages -all -p "$pdf" "$output_dir/img"
        local count=$(find "$output_dir" -type f | wc -l)
        echo "  Images: $count"
    else
        echo "WARNING: PDF not found: $pdf"
    fi
}

# Main
main() {
    echo "=== Car Wars Book Extraction Script ==="
    echo "Project root: $PROJECT_ROOT"
    echo "Books directory: $BOOKS_DIR"
    echo ""

    check_deps

    # Check if PDFs exist
    if [ ! -d "$BOOKS_DIR" ]; then
        echo "ERROR: Books directory not found: $BOOKS_DIR"
        echo "Please place the PDF files in the books/ directory:"
        echo "  - Car_Wars_Compendium.pdf"
        echo "  - Uncle_Albert_s_Catalog_from_Hell.pdf"
        echo "  - Car_Wars_Deluxe_Box_Material.pdf"
        exit 1
    fi

    # Create image directories
    mkdir -p "$BOOKS_DIR/images/compendium"
    mkdir -p "$BOOKS_DIR/images/uncle_alberts"
    mkdir -p "$BOOKS_DIR/images/deluxe"

    echo "--- Extracting Text ---"
    extract_text "$BOOKS_DIR/Car_Wars_Compendium.pdf" "$BOOKS_DIR/Car_Wars_Compendium.txt"
    extract_text "$BOOKS_DIR/Uncle_Albert_s_Catalog_from_Hell.pdf" "$BOOKS_DIR/Uncle_Alberts_Catalog.txt"
    extract_text "$BOOKS_DIR/Car_Wars_Deluxe_Box_Material.pdf" "$BOOKS_DIR/Car_Wars_Deluxe.txt"

    echo ""
    echo "--- Extracting Images ---"
    extract_images "$BOOKS_DIR/Car_Wars_Compendium.pdf" "$BOOKS_DIR/images/compendium"
    extract_images "$BOOKS_DIR/Uncle_Albert_s_Catalog_from_Hell.pdf" "$BOOKS_DIR/images/uncle_alberts"
    extract_images "$BOOKS_DIR/Car_Wars_Deluxe_Box_Material.pdf" "$BOOKS_DIR/images/deluxe"

    echo ""
    echo "=== Extraction Complete ==="
    echo ""
    echo "Text files:"
    ls -lh "$BOOKS_DIR"/*.txt 2>/dev/null || echo "  (none)"
    echo ""
    echo "Image directories:"
    du -sh "$BOOKS_DIR/images"/* 2>/dev/null || echo "  (none)"
}

main "$@"
