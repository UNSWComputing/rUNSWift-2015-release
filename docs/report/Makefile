default: report

# intentionally run 3 times, thanks to the way bibtex works, needed to regen TOC
report:report.tex report.bib
	pdflatex report
	bibtex report
	pdflatex report
	bibtex report
	pdflatex report

clean:
	rm report.aux report.bbl report.blg report.log report.pdf report.toc

