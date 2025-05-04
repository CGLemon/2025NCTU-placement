# --- 編譯器 ---
CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -I. -IBStarTree   # ❶ include 路徑改成 . 及子資料夾

# --- 來源檔 (.cpp 全在目前資料夾) ---
SRC      := $(wildcard *.cpp)                      # ❷ 去掉 src/ 前綴

TARGET   := ../bin/hw4

all: $(TARGET)

$(TARGET): $(SRC)
	@mkdir -p ../bin
	$(CXX) $(CXXFLAGS) $^ -o $@                   # ❸ 不要再加 -Isrc
