jz100 : mainBoard.o 8085.o 8088.o e8259.o e8253.o wd1797.o keyboard.o video.o screen.o debug_gui.o utility_functions.o
	gcc -pthread -o jz100 mainBoard.o 8085.o 8088.o e8259.o e8253.o wd1797.o keyboard.o video.o screen.o debug_gui.o utility_functions.o `pkg-config --cflags --libs gtk+-3.0` -export-dynamic
mainBoard.o : mainBoard.c mainBoard.h 8085.h 8088.h e8259.h e8253.h wd1797.h keyboard.h video.h screen.h debug_gui.h utility_functions.h
	gcc -c mainBoard.c `pkg-config --cflags --libs gtk+-3.0` -export-dynamic
8085.o : 8085.c 8085.h mainBoard.h
	gcc -c 8085.c
8088.o : 8088.c 8088.h mainBoard.h
	gcc -c 8088.c
e8259.o : e8259.c e8259.h
	gcc -c e8259.c
e8253.o : e8253.c e8253.h
	gcc -c e8253.c
wd1797.o : wd1797.c wd1797.h
	gcc -c wd1797.c
keyboard.o : keyboard.c keyboard.h
	gcc -c keyboard.c
video.o : video.c video.h
	gcc -c video.c
screen.o : screen.c video.h screen.h keyboard.h mainBoard.h
	gcc -c screen.c `pkg-config --cflags --libs gtk+-3.0` -export-dynamic
debug_gui.o : debug_gui.c 8085.h debug_gui.h
	gcc -c debug_gui.c `pkg-config --cflags --libs gtk+-3.0` -export-dynamic
utility_functions.o : utility_functions.c
	gcc -c utility_functions.c
clean :
	rm jz100 mainBoard.o 8085.o 8088.o e8259.o e8253.o wd1797.o keyboard.o debug_gui.o utility_functions.o video.o screen.o

# The make lines below is a replica of the above lines except they
# include the jumper compilation

# jz100 : mainBoard.o 8085.o 8088.o jumper.o
#  gcc -o jz100 mainBoard.o 8085.o 8088.o jumper.o
# mainBoard.o : mainBoard.c mainBoard.h jumper.h 8085.h 8088.h
#  gcc -c mainBoard.c
# 8085.o : 8085.c 8085.h mainBoard.h
#  gcc -c 8085.c
# 8088.o : 8088.c 8088.h mainBoard.h
#	gcc -c 8088.c
# jumper.o : jumper.c jumper.h
#	gcc -c jumper.c
# clean :
#	rm jz100 mainBoard.o 8085.o 8088.o jumper.o
