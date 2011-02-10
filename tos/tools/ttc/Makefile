
SRC = \
	src/org/tos/ttc/Main.java \
	src/org/tos/ttc/Buttons.java \
	src/org/tos/ttc/Config.java \
	src/org/tos/ttc/CustomJTextPane.java \
	src/org/tos/ttc/ErrorCode.java \
	src/org/tos/ttc/Resource.java \
	src/org/tos/ttc/Screenshot.java \
	src/org/tos/ttc/SourceView.java \
	src/org/tos/ttc/TableModel.java \
	src/org/tos/ttc/TestCases.java \
	src/org/tos/ttc/TestConsole.java

all: ttc.jar

ttc.jar: $(SRC)
	javac -source 1.4 -d . -sourcepath ./src src/org/tos/ttc/Main.java
	jar cfm ttc.jar MANIFEST.MF org/tos/ttc/*.class images/* xml/*
	rm -rf org
	cp ttc.jar ../..

clean:
	rm -f ttc.jar

depend:


