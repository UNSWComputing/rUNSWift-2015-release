all: paper.pdf paperVer2.pdf

# intentionally run 3 times, thanks to the way bibtex works, needed to regen TOC
%.pdf:%.tex 
	pdflatex $*
	bibtex $*
	pdflatex $*
	bibtex $*
	pdflatex $*

clean:
	rm -f *.aux *.bbl *.blg *.log *.pdf *.toc
