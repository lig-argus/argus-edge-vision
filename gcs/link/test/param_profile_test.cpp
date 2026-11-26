#include "argus/gcs/link/param_profile.hpp"

#include <QtTest>
#include <cstdint>
#include <vector>

#include "argus/mavlink.h"

namespace argus::gcs::link {

class ParamProfileTest : public QObject {
    Q_OBJECT

private slots:
    // 주석과 빈 줄은 건너뛰고, 같은 이름은 뒤의 값 하나만 남긴다
    void parsesCommentsAndDuplicates() {
        QString error;
        const auto profile = parseParamProfile(FlightEnv::Indoor,
                                               QStringLiteral("# 주석\n\n"
                                                              "1\t1\tSYS_HAS_GPS\t1\t6\n"
                                                              "1\t1\tEKF2_OF_POS_X\t0.075\t9\n"
                                                              "1\t1\tSYS_HAS_GPS\t0\t6\n"),
                                               &error);
        QVERIFY2(profile.has_value(), qPrintable(error));
        const ParamProfile parsed = profile.value_or(ParamProfile{});
        QCOMPARE(parsed.env, FlightEnv::Indoor);
        const std::vector<ParamValue>& values = parsed.values;
        QCOMPARE(values.size(), std::size_t{2});
        QCOMPARE(values.at(0).name, QStringLiteral("SYS_HAS_GPS"));
        QCOMPARE(values.at(0).value, 0.0);
        QCOMPARE(values.at(1).type, std::uint8_t{MAV_PARAM_TYPE_REAL32});
    }

    // UTF-8 바이트 글도 같게 읽는다 (CRLF 줄 끝 포함)
    void parsesUtf8Bytes() {
        QString error;
        const auto profile =
            parseParamProfile(FlightEnv::Outdoor,
                              QByteArrayLiteral("# 실외\r\n1\t1\tEKF2_HGT_REF\t2\t6\r\n"), &error);
        QVERIFY2(profile.has_value(), qPrintable(error));
        const ParamProfile parsed = profile.value_or(ParamProfile{});
        QCOMPARE(parsed.env, FlightEnv::Outdoor);
        QCOMPARE(parsed.values.size(), std::size_t{1});
        QCOMPARE(parsed.values.at(0).value, 2.0);
    }

    // 칸 수, 형식 번호(INT32, REAL32 만), 이름 길이(16 자)가 틀리면 줄 번호와 함께 거절
    void rejectsMalformedLine() {
        QString error;
        QVERIFY(!parseParamProfile(FlightEnv::Indoor, QStringLiteral("1\t1\tSYS_HAS_GPS\t1\n"),
                                   &error));
        QVERIFY(error.startsWith(QStringLiteral("1번째 줄")));
        QVERIFY(!parseParamProfile(FlightEnv::Indoor,
                                   QStringLiteral("# a\n1\t1\tSYS_HAS_GPS\t1\t5\n"), &error));
        QVERIFY(error.startsWith(QStringLiteral("2번째 줄")));
        QVERIFY(!parseParamProfile(FlightEnv::Indoor,
                                   QStringLiteral("1\t1\tABCDEFGHIJKLMNOPQ\t1\t6\n"), &error));
        QVERIFY(!parseParamProfile(FlightEnv::Indoor, QStringLiteral("1\t1\tA\tx\t6\n"), &error));
    }

    // 이름 16 자는 받는다 (param_id 의 최대 길이)
    void acceptsSixteenCharacterName() {
        QString error;
        const auto profile = parseParamProfile(
            FlightEnv::Indoor, QStringLiteral("1\t1\tABCDEFGHIJKLMNOP\t1\t6\n"), &error);
        QVERIFY2(profile.has_value(), qPrintable(error));
    }

    // FC 가 돌려주는 float 값(0.075000003)은 파일 값(0.075)과 같다. INT32 는 정확히 비교
    void comparesWithFloatPrecision() {
        QVERIFY(sameParamValue(MAV_PARAM_TYPE_REAL32, 0.075, 0.07500000298023224));
        QVERIFY(!sameParamValue(MAV_PARAM_TYPE_REAL32, 0.075, 0.08));
        QVERIFY(sameParamValue(MAV_PARAM_TYPE_INT32, 2.0, 2.0));
        QVERIFY(!sameParamValue(MAV_PARAM_TYPE_INT32, 2.0, 1.0));
    }

    void namesEnvironments() {
        QCOMPARE(flightEnvName(FlightEnv::Indoor), QStringLiteral("실내"));
        QCOMPARE(flightEnvName(FlightEnv::Outdoor), QStringLiteral("실외"));
    }
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::ParamProfileTest)
#include "param_profile_test.moc"
