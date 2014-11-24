#!/bin/bash

SRC=main

#rm -rf out &> /dev/null
mkdir out &> /dev/null
cp -fP *.bib out/ &> /dev/null

rm out/$SRC.pdf

pdflatex --halt-on-error --shell-escape -output-directory=out ../$SRC.tex
cd out
biber $SRC
cd ..
pdflatex --halt-on-error --shell-escape -output-directory=out ../$SRC.tex
pdflatex --halt-on-error --shell-escape -output-directory=out ../$SRC.tex

if [ -f out/$SRC.pdf ]
then
    (xdg-open out/$SRC.pdf &)
fi

# TODO: This is bad
rm *.pdf*
