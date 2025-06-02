all: gctest

gctest:
	gcc -I . -I ./dummy_incdir -I .. -m32 -o gctest pc_test_main.c ../gyrocompass.c gyrocompass_test.c -lm

run: gctest
	./gctest ../../cxd5602pwbimu_logger/pc_tools/gyrocompass/sample_data

clean:
	rm -f gctest
