# urgwidget

VERSION = 1.2.5
RELEASE_DIR = release
PACKAGE_EN_DIR = urg_library-$(VERSION)
PACKAGE_JA_DIR = urg_library_ja-$(VERSION)

PWD = $(shell pwd)

all :
	cd current/ && $(MAKE)

clean : release_clean
	cd current/ && $(MAKE) clean
	$(RM) -rf $(RELEASE_DIR)

install : dist
	cd $(RELEASE_DIR)/$(PACKAGE_JA_DIR) && $(MAKE) install


TARGET_DIR = $(PACKAGE_EN_DIR) $(PACKAGE_JA_DIR)
dist : release_clean
	mkdir -p $(RELEASE_DIR)
	for i in $(TARGET_DIR) ; \
	do \
		mkdir -p $(RELEASE_DIR)/$$i; \
		mkdir -p $(RELEASE_DIR)/$$i/include; \
		mkdir -p $(RELEASE_DIR)/$$i/include/c; \
		mkdir -p $(RELEASE_DIR)/$$i/include/cpp; \
		mkdir -p $(RELEASE_DIR)/$$i/src; \
		mkdir -p $(RELEASE_DIR)/$$i/windowsexe; \
		mkdir -p $(RELEASE_DIR)/$$i/vs2005; \
		mkdir -p $(RELEASE_DIR)/$$i/vs2010; \
		mkdir -p $(RELEASE_DIR)/$$i/vs2015; \
		mkdir -p $(RELEASE_DIR)/$$i/vs2017; \
		mkdir -p $(RELEASE_DIR)/$$i/samples; \
		mkdir -p $(RELEASE_DIR)/$$i/samples/c; \
		mkdir -p $(RELEASE_DIR)/$$i/samples/cpp; \
		cp current/COPYRIGHT.txt current/Install.txt current/build_rule.mk Readme.txt AUTHORS.txt Releasenotes.txt $(RELEASE_DIR)/$$i/; \
		cp current/Makefile.release $(RELEASE_DIR)/$$i/Makefile; \
		cp current/src/Makefile.release $(RELEASE_DIR)/$$i/src/Makefile; \
		cp current/samples/Makefile $(RELEASE_DIR)/$$i/samples/Makefile ; \
		cp current/samples/c/Makefile.release $(RELEASE_DIR)/$$i/samples/c/Makefile ; \
		cp current/samples/cpp/Makefile.release $(RELEASE_DIR)/$$i/samples/cpp/Makefile ; \
		cp current/include/c/*.h $(RELEASE_DIR)/$$i/include/c/; \
		cp current/include/cpp/*.h $(RELEASE_DIR)/$$i/include/cpp/; \
		cp current/src/*.c $(RELEASE_DIR)/$$i/src/; \
		cp current/src/*.cpp $(RELEASE_DIR)/$$i/src/; \
		cp current/samples/c/*.sh $(RELEASE_DIR)/$$i/samples/c/; \
		cp current/windowsexe/*.bat $(RELEASE_DIR)/$$i/windowsexe/; \
		cp -r current/vs2005/ $(RELEASE_DIR)/$$i/; \
		cp -r current/vs2010/ $(RELEASE_DIR)/$$i/; \
		cp -r current/vs2015/ $(RELEASE_DIR)/$$i/; \
		cp -r current/vs2017/ $(RELEASE_DIR)/$$i/; \
		cat current/urg_c-config.in | sed -e "s/VERSION/$(VERSION)/g" > $(RELEASE_DIR)/$$i/urg_c-config.in ; \
		cat current/urg_cpp-config.in | sed -e "s/VERSION/$(VERSION)/g" > $(RELEASE_DIR)/$$i/urg_cpp-config.in ; \
	done
	ruby split_comment.rb -e current/include/c/*.h $(RELEASE_DIR)/$(PACKAGE_EN_DIR)/include/c/
	ruby split_comment.rb -e current/include/cpp/*.h $(RELEASE_DIR)/$(PACKAGE_EN_DIR)/include/cpp/
	ruby split_comment.rb -e current/src/*.c $(RELEASE_DIR)/$(PACKAGE_EN_DIR)/src/
	ruby split_comment.rb -e current/src/*.cpp $(RELEASE_DIR)/$(PACKAGE_EN_DIR)/src/
	ruby split_comment.rb -e current/samples/c/*.c $(RELEASE_DIR)/$(PACKAGE_EN_DIR)/samples/c/
	ruby split_comment.rb -e current/samples/cpp/*.cpp $(RELEASE_DIR)/$(PACKAGE_EN_DIR)/samples/cpp/
	ruby split_comment.rb -e current/samples/c/*.h $(RELEASE_DIR)/$(PACKAGE_EN_DIR)/samples/c/
	ruby split_comment.rb -e current/samples/cpp/*.h $(RELEASE_DIR)/$(PACKAGE_EN_DIR)/samples/cpp/
	ruby split_comment.rb -j current/include/c/*.h $(RELEASE_DIR)/$(PACKAGE_JA_DIR)/include/c/
	ruby split_comment.rb -j current/include/cpp/*.h $(RELEASE_DIR)/$(PACKAGE_JA_DIR)/include/cpp/
	ruby split_comment.rb -j current/src/*.c $(RELEASE_DIR)/$(PACKAGE_JA_DIR)/src/
	ruby split_comment.rb -j current/src/*.cpp $(RELEASE_DIR)/$(PACKAGE_JA_DIR)/src/
	ruby split_comment.rb -j current/samples/c/*.c $(RELEASE_DIR)/$(PACKAGE_JA_DIR)/samples/c/
	ruby split_comment.rb -j current/samples/cpp/*.cpp $(RELEASE_DIR)/$(PACKAGE_JA_DIR)/samples/cpp/
	ruby split_comment.rb -j current/samples/c/*.h $(RELEASE_DIR)/$(PACKAGE_JA_DIR)/samples/c/
	ruby split_comment.rb -j current/samples/cpp/*.h $(RELEASE_DIR)/$(PACKAGE_JA_DIR)/samples/cpp/

	for i in $(TARGET_DIR) ; \
	do \
		cd $(PWD); \
		cd $(RELEASE_DIR)/$$i && $(MAKE) && $(MAKE) clean; \
	done
	for i in $(TARGET_DIR) ; \
	do \
		cd $(PWD); \
		cd $(RELEASE_DIR)/ && (zip -r $$i.zip $$i) && mv $$i.zip ../; \
	done

release_clean :
	$(RM) -rf $(RELEASE_DIR)/$(PACKAGE_EN_DIR) $(RELEASE_DIR)/$(PACKAGE_JA_DIR)
	$(RM) $(PACKAGE_EN_DIR).zip $(PACKAGE_JA_DIR).zip
