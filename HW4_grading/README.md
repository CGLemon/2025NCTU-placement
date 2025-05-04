# Homework 4 Grading Script
We will use this script to grade your program. **Make sure your program can be executed by this script.**

## Preparing
* Step1:  
    Enter the `HW4_grading` directory and create a new directory named with your student ID in the `student` directory.
    ```sh
    $ cd HW4_grading/
    $ mkdir student/${your_student_id}
    ```
    For example,
    ```sh
    $ cd HW4_grading/
    $ mkdir student/113000000
    ```

* Step2:  
    Put your compressed file in the directory which you just created.  
    The correct path should be:
    ```
    HW4_grading/student/${your_student_id}/CS6135_HW4_${your_student_id}.tar.gz
    ```
    For example,
    ```
    HW4_grading/student/113000000/CS6135_HW4_113000000.tar.gz
    ```

### Notice:  
**Please make sure not to put your original directory here**, as it will remove all directories before unzipping the compressed file.

## Grading
* Step1:  
    Navigate to the `HW4_grading` directory and run `HW4_grading.sh`.
    ```sh
    $ cd HW4_grading/
    $ bash HW4_grading.sh
    ```

* Step3:  
    Check your output. Ensure the status of each checking item is **yes**.
    * If the status of a testcase is **success**, it means your program finished in time, and the output result is legal.
        ```
        host name: ic22
        compiler version: g++ (GCC) 9.3.0
        
        grading on 113000000:
         checking item            | status
        --------------------------|--------
         correct tar.gz           | yes
         correct file structure   | yes
         have README              | yes
         have Makefile            | yes
         correct make clean       | yes
         correct make             | yes
        
          testcase |         area |    runtime | status
        -----------|--------------|------------|--------
           public1 |     52660568 |      10.82 | success
           public2 |       695913 |       9.40 | success
           public3 |       636576 |      13.75 | success
        ```

    * If the status of a testcase is not **success**, it means your program failed in this testcase.
        ```
        host name: ic22
        compiler version: g++ (GCC) 9.3.0
        
        grading on 113000000:
         checking item            | status
        --------------------------|--------
         correct tar.gz           | yes
         correct file structure   | yes
         have README              | yes
         have Makefile            | yes
         correct make clean       | yes
         correct make             | yes
        
          testcase |         area |    runtime | status
        -----------|--------------|------------|--------
           public1 |          N/A |        TLE | Time out while testing public1.
           public2 |          N/A |        TLE | Time out while testing public2.
           public3 |          N/A |        TLE | Time out while testing public3.
        ```
