#!/bin/env/python

import os, sys
import glob
from shutil import copy

def main(dest):
	try:
		os.mkdir(dest)
	except OSError as e:
		print(e)
	try:
		os.mkdir(dest + "/Code")
	except OSError as e:
		print(e)
	with open("codeToLaTeX.tex", "w+") as f:
		f.write("%%-----------------------------------------\n")
		f.write("\\section{Header Files}\n\n")
		for header in glob.glob('**/*.h*', recursive=True):
			if "api.h" in header or "/display/" in header or "/Eigen/" in header or "/okapi/" in header or "/pros/" in header:
				continue
			f.write("\\subsection{" + header + "}\n")
			f.write("\\inputminted{c}{Code/" + header + "}\n")
			f.write("\\pagebreak\n\n")
			head, tail = os.path.split(header)
			path = dest + "/Code" + "/" + head
			try:
				os.mkdir(path)
				print(path)
			except OSError as e:
				print(e)
			copy(header, path)

		f.write("%%-----------------------------------------\n")
		f.write("\\section{Source Files}\n\n")

		for source in glob.glob('**/*.c*', recursive=True):
			if ".o" in source or ".csv" in source:
				continue
			f.write("\\subsection{" + source + "}\n")
			f.write("\\inputminted{c}{Code/" + source + "}\n")
			f.write("\\pagebreak\n\n")
			head, tail = os.path.split(source)
			path = dest + "/Code" + "/" + head
			try:
				os.mkdir(path)
			except OSError as e:
				print(e)
			copy(source, path)

	copy("codeToLaTeX.tex", dest)
	os.remove("codeToLaTeX.tex")
	with open("main.tex", "w+") as f:
		f.write("\\documentclass{article}\n\\usepackage[utf8]{inputenc}\n\\usepackage[margin=1in]{geometry}\n\\title{test1}\n\\author{baylessj}\n\\date{April 2019}\n\\usepackage{minted}\n\\begin{document}\n\\input{codeToLaTeX.tex}\n\n\\end{document}")
	copy("main.tex", dest)
	os.remove("main.tex")

if __name__ == "__main__":
   main(sys.argv[1])
