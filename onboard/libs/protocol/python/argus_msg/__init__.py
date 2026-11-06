# 프로세스 사이 메시지(docs/spec/process_interfaces.md 5장)의 Python protobuf 코드.
# *_pb2.py, *_pb2.pyi 는 tools/generate_proto.sh 가 ../../proto/argus_msg/*.proto 에서
# 생성한다. 직접 수정하지 않는다.
# 사용하는 쪽은 PYTHONPATH 에 onboard/libs/protocol/python 을 넣고
# `from argus_msg import control_setpoint_pb2` 로 부른다.
# 런타임 버전은 requirements.txt.
