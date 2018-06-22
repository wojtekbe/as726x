#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char* argv[]) {
	FILE *f;
	char cmd[256];
	int result_i;
	float result_f;

	if (argc < 2)
		return -1;

	sprintf(cmd, "cat %s", argv[1]);
	f = popen(cmd, "r");
	fscanf (f, "%d", &result_i);
	memcpy((void*)&result_f, &result_i, 4);
	//printf("%d -> %f\n", result_i, result_f);
	printf("%f\n", result_f);
	pclose(f);

	return 0;
}
