#include "model_change.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	model_change w;
	w.show();
	return a.exec();
}
