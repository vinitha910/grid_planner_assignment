all:
	g++ -std=c++11 -c test_planner.cpp graph.h graph.cpp dijkstra.h dijkstra.cpp
	g++ -std=c++11 -o test test_planner.o graph.o dijkstra.o -ljsoncpp

test: main.o graph.o dijkstra.o
	g++ -std=c++11 -Wall main.o graph.o dijkstra.o -o test

main.o: test_planner.cpp
	g++ -std=c++11 -c test_planner.cpp

graph.o: graph.cpp graph.h
	g++ -std=c++11 -c graph.cpp

dijkstra.o: dijkstra.cpp dijkstra.h
	g++ -std=c++11 -c dijkstra.cpp

