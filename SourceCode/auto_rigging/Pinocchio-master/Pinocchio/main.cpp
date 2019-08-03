//

#include "pinocchioApi.h"

int main(int argc, char** args)
{
	HumanSkeleton ske;
	Mesh mesh(args[1]);
	auto out = autorig(ske, mesh, args[1]);

	system("pause");
	return 0;
}