#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main() {
	FILE *f;
	char cmd[256];
	int idx;
	int result_i;
	float result_f;

	for (idx=0; idx<6; idx++) {
		sprintf(cmd, "/sys/bus/iio/devices/iio:device0/in_intensity%d_input", idx);
		f = fopen(cmd, "r");
		fscanf (f, "%d", &result_i);
		memcpy((void*)&result_f, &result_i, 4);
		printf("%d: %d %f\n", idx, result_i, result_f);
		fclose(f);
	}
	return 0;
}
