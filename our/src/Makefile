# --- 編譯器 ---
CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -I. -IBStarTree

# --- 來源檔 (.cpp 全在目前資料夾) ---
SRCS     := $(wildcard *.cpp)
TARGET   := ../bin/hw4

release:
	@mkdir -p ../bin
	$(CXX) $(SRCS) -o $(TARGET) $(CXXFLAGS)