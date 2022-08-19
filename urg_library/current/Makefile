# Makefile for urgwidget

OUTPUT_HTML_DIR = html
OUTPUT_HTML_JA_DIR = html_ja

all :
	cd src/ && $(MAKE)
	cd samples/ && $(MAKE)

clean :
	$(RM) -rf html html_ja $(OUTPUT_HTML_DIR) $(OUTPUT_HTML_JA_DIR)
	cd src/ && $(MAKE) clean
	cd samples/ && $(MAKE) clean
	-cd viewers/ && $(MAKE) clean
	cd dox/ && $(MAKE) clean

html :
	cd dox/ && $(MAKE) img
	doxygen Doxyfile_ja
	cp COPYRIGHT.txt $(OUTPUT_HTML_JA_DIR)
	doxygen Doxyfile
	cp COPYRIGHT.txt $(OUTPUT_HTML_DIR)

.PHONY : all clean html
######################################################################
