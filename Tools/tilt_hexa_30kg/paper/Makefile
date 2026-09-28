# =====================================================================
# Makefile for the 30 kg fully-tilting hexacopter manuscript
# Target journal: Aerospace Science and Technology (Elsevier)
# Usage (from this paper/ directory):
#   make figures   # regenerate EVERY figure from versioned data + scripts
#   make paper     # compile main.pdf (pdflatex + bibtex, English)
#   make zh        # compile main_zh.pdf (xelatex + bibtex, Chinese)
#   make all       # figures then paper
#   make clean     # remove LaTeX build artifacts
# =====================================================================

PY  ?= /opt/python3.12/bin/python3
PDF := main.pdf
ZH  := main_zh.pdf

.PHONY: all figures paper zh clean

all: figures paper

# All figures (AFMS, transition/comparison, corridor/trim/profile/AWS/
# robustness/solve-time/Monte-Carlo, and the real-arduplane SITL figures)
# are rebuilt from the committed result data by rebuild_figures.sh.
figures:
	bash rebuild_figures.sh

paper: $(PDF)

$(PDF): main.tex references.bib
	pdflatex -interaction=nonstopmode main
	-bibtex main
	pdflatex -interaction=nonstopmode main
	pdflatex -interaction=nonstopmode main

# Chinese version: ctexart + xelatex (a CJK-capable engine is required).
# It shares the same figures/ and references.bib with the English version.
zh: $(ZH)

$(ZH): main_zh.tex references.bib
	xelatex -interaction=nonstopmode main_zh
	-bibtex main_zh
	xelatex -interaction=nonstopmode main_zh
	xelatex -interaction=nonstopmode main_zh

clean:
	rm -f main.aux main.bbl main.blg main.log main.out main.spl main.pdf
	rm -f main_zh.aux main_zh.bbl main_zh.blg main_zh.log main_zh.out main_zh.pdf
