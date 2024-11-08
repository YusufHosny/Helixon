## IB3 Paper LaTeX template
This repo holds a IB3 template for the FIIW. The `fiiw.sty` file provides the package with all layout specifications. All other files are a preview implementation for a IB3 paper.

### Usage
Include `\usepackage[groept]{fiiw}` in your main tex file. For usage on an other campus replace replace `groept` with your campus name. Curently campus `groept`, `denayer`, `gent`, `geel` and `brugge` are supported.
Extra options can be added like : `noacknowledgements`, `noabstract`, `noextendedabstract`, `nolistoffigures`, `nolistoftables`, `nolistofsymbols`
The fiiw-package expects some basic information like thesis title, student names,... to be set. Check the basic example below or the preview implementation for further info.
The main document language can be set with `\documentlanguage{}`, optional you can specify a different language for the cover (e.g. dutch students writing their text in english) with `\coverlanguage{}`

#### Basic example (check template documents)
```latex
\documentclass[11pt,a4paper,oneside]{book}
\usepackage[groept,nolistofsymbols]{fiiw}

%% .tex file with acknowledgements
\acknowledgementsfile{chapters/acknowledgements}
%% .tex file with abstract
\abstractENfile{chapters/abstract}
%% .tex file with list of symbols
\listofsymbolsfile{chapters/symbols}

\documentlanguage{english}

\program{Electronics Engineering}
\title{Thesis Title}
\subtitle{Thesis Subtitle (optional)}
\firstnameA{Firstname}
\lastnameA{Lastname}
\firstnameB{Firstname2}
\lastnameB{Lastname2}
\academicyear{2021 - 2022}
\supervisor{My Supervisor}
\supervisorEmail{My.supervisor@KULeuven.be}

\begin{document}
  \preface

  ...

  \backcover
\end{document}
```
