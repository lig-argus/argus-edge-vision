// argus_gcs: 지상 통제 프로그램 (Qt Widgets). 시작점만 있다.
// 통신·화면은 docs/ICD.md 순서로 추가한다.
#include <QApplication>
#include <QMainWindow>

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    QMainWindow window;
    window.setWindowTitle(QStringLiteral("argus_gcs"));
    window.resize(1280, 720);
    window.show();
    return QApplication::exec();
}
