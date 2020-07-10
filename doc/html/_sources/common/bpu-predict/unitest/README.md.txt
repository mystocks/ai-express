for BPU predict unitest
============================================

compile:
---------
build libbpu_predict.so:

build bpu_predict_unitest

run unitest
---------
copy libbpu_predict.so to runtime/lib
copy bpu_predict_unitest to runtime/

copy the whole runtime dir to x2/j2 dev board

execute:
sh run-test.sh

