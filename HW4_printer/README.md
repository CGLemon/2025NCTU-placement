# Homework 4 Printer
You can use this program to visualize your placement result.

#
## Preparing
* __Step1:__  
    This placement visualization tool is written in __Processing__.  
    Please download the [__Processing__](https://processing.org/download) and unzip it.

* __Step2:__  
    Unzip `HW4_printer.zip` and put this directory in the folder `processing`.  
    The file hierarchy is shown as below:

    ```
    processing/
    |   core/
    |   HW4_printer/
    |   |   figure/
    |   |   output/
    |   |   testcase/
    |   |   |   public1.txt
    |   |   |   public2.txt
    |   |   |   public3.txt
    |   |   HW4_printer.pde
    |   |   README.md
    |   java/
    |   lib/
    |   modes/
    |   tools/
    |   processing.exe
    |   processing-java.exe
    |   revisions.md
    ```

* __Step3:__  
    Put your ww result in the folder `output`.

    ```
    processing/
    |   HW4_printer/
    |   |   output/
    |   |   |   public1.out
    |   |   |   public2.out
    |   |   |   public3.out
    |   |   ...
    ```

#
## Working Item
* __Step1:__  
    Open `HW4_printer.pde` with `processing.exe`.  
    Modify the *caseName* and *deadSpaceRatio* in __setup()__.

    ```java
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
    ```

    * __Notice:__   
        The default image size is 750 x 750.  
        If you want to change the image size, please revise the value of `imageSize` and `size(x, x)`.

* __Step2:__  
    Press __run__ button to execute the drawing program.  
    A drawing window will pop up on your screen.  
    The floorplanning image will be saved in the folder `figure`.

    ```
    processing/
    |   HW4_printer/
    |   |   figure/
    |   |   |   public1.png
    |   |   |   public2.png
    |   |   |   public3.png
    |   |   ...
    ```

#
## Reference
* https://processing.org/reference/
