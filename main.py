import asyncio
import argparse
import cProfile

from modules.utils import set_config
from modules.utils import config
from modules.bt_runner import BTRunner

# === 너의 QR / Nav2 노드 import ===
from scenarios.final_project.limo_nav_action_server import LimoNavigateServer
from scenarios.final_project.r_blackboard_updater import QrBlackboardUpdater

# QR Router (필요하면 켜기)
from limo_qr_system.qr_order_router_node import QrOrderRouter


async def loop(bt_runner):

    # ros executor 가져오기
    executor = bt_runner.agent.ros_bridge.executor

    while bt_runner.running:

        # executor 한 번 스핀 (ROS 콜백 수행)
        executor.spin_once(timeout_sec=0.01)

        # 키보드 이벤트 처리
        bt_runner.handle_keyboard_events()

        # BT Step
        if not bt_runner.paused:
            await bt_runner.step()

        # 화면 렌더링
        bt_runner.render()

    bt_runner.close()


def main():
    # Load config
    parser = argparse.ArgumentParser(description='py_bt_ros')
    parser.add_argument('--config', type=str, default='config.yaml')
    args = parser.parse_args()

    set_config(args.config)

    # === BT Runner 생성 ===
    bt_runner = BTRunner(config)
    executor = bt_runner.agent.ros_bridge.executor

    # === Nav2 브리지 노드 추가 ===
    nav_node = LimoNavigateServer(ns="/limo")
    executor.add_node(nav_node)

    # === QR blackboard updater 추가 ===
    qr_bb_node = QrBlackboardUpdater(bt_runner.blackboard)
    executor.add_node(qr_bb_node)

    # === QR Router 추가 (너가 QR 라우터 패키지 켤 때만) ===
    try:
        qr_router = QrOrderRouter()
        executor.add_node(qr_router)
    except Exception:
        print("[WARN] QR Router 패키지가 없어서 건너뜀")

    # === 메인 루프 ===
    if config['bt_runner']['profiling_mode']:
        cProfile.run('asyncio.run(loop(bt_runner))', sort='cumulative')
    else:
        asyncio.run(loop(bt_runner))


if __name__ == '__main__':
    main()
