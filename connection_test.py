from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time


def test_connection():
    try:
        print("CoppeliaSim 연결 시도...")
        client = RemoteAPIClient()
        sim = client.getObject('sim')
        print("연결 성공")

        sim.startSimulation()
        for i in range(3):
            print(f"시뮬레이션 시간: {sim.getSimulationTime():.2f}s")
            time.sleep(1)
        sim.stopSimulation()
        print("통신 테스트 완료")

    except Exception as e:
        print(f"연결 실패: {e}")
        print("CoppeliaSim이 실행 중인지 확인하세요.")


if __name__ == '__main__':
    test_connection()
