BIN=triangle-shooter

all:
	g++ -g -lSDL2 -lm -o $(BIN) *.cpp

run: $(BIN)
	./$(BIN)

debug:
	gdb $(BIN)
