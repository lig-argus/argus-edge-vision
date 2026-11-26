// argus_gcs: 지상 통제 프로그램 (Qt Quick). 시작점만 있다.
// 이 모듈(app)만 구체 구현을 만들어 다른 모듈에 넘긴다. 다른 모듈은 인터페이스만 받는다.
#include <QGuiApplication>
#include <QString>
#include <QWindow>

int main(int argc, char* argv[]) {
    const QGuiApplication app(argc, argv);
    QWindow window;
    window.setTitle(QStringLiteral("argus_gcs"));
    window.resize(1280, 720);
    window.show();
    return QGuiApplication::exec();
}
