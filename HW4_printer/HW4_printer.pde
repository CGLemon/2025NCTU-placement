/* ||=========================================|| */
/* || PDA HW4 placement visualization program || */
/* ||=========================================|| */

void setup() {
    String caseName = "public1"; // change testcase
    
    int imageSize = 750;         // change image size
    size(750, 750);              // should be same as imageSize
    background(255);
    
    Printer printer = new Printer(imageSize);
    printer.readInput("testcase/" + caseName + ".txt");
    printer.readOutput("output/" + caseName + ".out");
    printer.drawPlacement("figure/" + caseName + ".png");
}

class Hardblock {
    String name;
    int w, h;
    int x, y;
    color c;
    
    Hardblock(String name_, int w_, int h_) {
        name = name_;
        w = w_;
        h = h_;
        x = 0;
        y = 0;
        c = 180;
    }
    
    void setPosition(int x_, int y_) {
        x = x_;
        y = y_;
    }

    void rotate() {
        int temp = w;
        w = h;
        h = temp;
    }
}

class Printer {
    int imageSize;
    ArrayList<Hardblock> hardblocks = new ArrayList<Hardblock>();
    IntDict strToHardblockIdx = new IntDict();
    
    Printer(int imageSize_) {
        imageSize = imageSize_;
    }

    void readInput(String filename) {
        try {
            color c = 180;
            int ci = 0;
            color[] palette = {
                color(191, 154, 124),
                color(128, 105, 94),
                color(102, 122, 120),
                color(176, 192, 179),
                color(240, 234, 214)
            };

            BufferedReader reader = createReader(filename);
            String line;
            while ((line = reader.readLine()) != null) {
                String[] data = splitTokens(line, " ");
                if (data.length == 0) {
                    continue;
                }
                else if (data[0].equals("HardBlock")) {
                    String name = data[1];
                    int w = int(data[2]);
                    int h = int(data[3]);
                    strToHardblockIdx.set(name, hardblocks.size());
                    hardblocks.add(new Hardblock(name, w, h));
                }
                else if (data[0].equals("SymGroup")) {
                    c = palette[ci++];
                }
                else if (data[0].equals("SymPair")) {
                    hardblocks.get(strToHardblockIdx.get(data[1])).c = c;
                    hardblocks.get(strToHardblockIdx.get(data[2])).c = c;
                }
                else if (data[0].equals("SymSelf")) {
                    hardblocks.get(strToHardblockIdx.get(data[1])).c = c;
                }
            }
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    void readOutput(String filename) {
        try {
            BufferedReader reader = createReader(filename);
            String line;
            while ((line = reader.readLine()) != null) {
                String[] data = splitTokens(line, " ");
                if (data.length == 4) {
                    String name = data[0];
                    int x = int(data[1]);
                    int y = int(data[2]);
                    hardblocks.get(strToHardblockIdx.get(name)).setPosition(x, y);
                    if (data[3].equals("1")) {
                        hardblocks.get(strToHardblockIdx.get(name)).rotate();
                    }
                }
            }
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    float calScaleFactor() {
        int maxX = 0, maxY = 0;
        for (Hardblock hardblock : hardblocks) {
            maxX = max(maxX, hardblock.x + hardblock.w);
            maxY = max(maxY, hardblock.y + hardblock.h);
        }
        return (float)(imageSize - 5) / max(maxX, maxY);
    }
    
    float calX(float x) {
        return x + 1;
    }
    
    float calY(float y, float h) {
        return imageSize - y - h - 2;
    }
    
    void drawHardblocks() {
        float scaleFactor = calScaleFactor();
        for (Hardblock hardblock : hardblocks) {
            float w = hardblock.w * scaleFactor;
            float h = hardblock.h * scaleFactor;
            float x = calX(hardblock.x * scaleFactor);
            float y = calY(hardblock.y * scaleFactor, h);

            fill(hardblock.c);
            stroke(0);
            rect(x, y, w, h);

            fill(0);
            textSize(8);
            text(hardblock.name, x + 2, y + 9);
        }
    }

    void drawBoundingBox() {
        float scaleFactor = calScaleFactor();
        int width = 0;
        int height = 0;
        for (Hardblock hardblock : hardblocks) {
            width = max(width, hardblock.x + hardblock.w);
            height = max(height, hardblock.y + hardblock.h);
        }

        float w = width * scaleFactor;
        float h = height * scaleFactor;
        float x = calX(0);
        float y = calY(0, h);
        stroke(255, 0, 0);
        noFill();
        strokeWeight(1);
        rect(x, y, w, h);
    }

    
    void drawPlacement(String filename) {
        drawHardblocks();
        drawBoundingBox();
        save(filename);
    }
}
