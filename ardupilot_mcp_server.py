from mcp.server.fastmcp import FastMCP
from pymavlink import mavutil
import time

# MCP初期化
mcp = FastMCP("ArduPilot Controller", debug=True)
print("MCPサーバー初期化完了")

# ArduPilot接続（TCP & 指定システム/コンポーネントID）
def connect_to_ardupilot():
    try:
        print("ArduPilotに接続中... (tcp:127.0.0.1:5762)")
        conn = mavutil.mavlink_connection(
            "tcp:127.0.0.1:5762",
            source_system=1,
            source_component=90,
            autoreconnect=True
        )
        print("接続オブジェクト作成完了、ハートビート待機中...")
        if not conn.wait_heartbeat(timeout=10):
            raise Exception("10秒間ハートビートを受信できませんでした")
        print(f"ArduPilotに接続しました (システムID: {conn.target_system}, コンポーネントID: {conn.target_component})")
        return conn
    except Exception as e:
        print(f"接続エラー: {str(e)}")
        print("接続設定を確認してください:")
        print("- SITL/実機が起動しているか")
        print("- ポート番号が正しいか (5762)")
        print("- ファイアウォール設定")
        raise

# アーム
@mcp.tool()
def arm() -> str:
    conn = connect_to_ardupilot()
    try:
        if not conn.wait_heartbeat(timeout=10):
            return "エラー: ArduPilotとの接続がタイムアウトしました"
        conn.arducopter_arm()
        conn.motors_armed_wait()
        return "機体をアームしました。"
    except Exception as e:
        return f"エラー: {str(e)}\n接続設定を確認してください:\n- SITL/実機が起動しているか\n- ポート番号が正しいか (5762)\n- ファイアウォール設定"
    finally:
        conn.close()

# ディスアーム
@mcp.tool()
def disarm() -> str:
    conn = connect_to_ardupilot()
    try:
        conn.wait_heartbeat()
        conn.arducopter_disarm()
        conn.motors_disarmed_wait()
        return "機体をディスアームしました。"
    finally:
        conn.close()

# 離陸（高度指定）
@mcp.tool()
def takeoff(altitude: float = 10.0) -> str:
    conn = connect_to_ardupilot()
    try:
        # ハートビート待機（タイムアウト10秒）
        if not conn.wait_heartbeat(timeout=10):
            return "エラー: ArduPilotとの接続がタイムアウトしました"

        # 現在のモードを確認
        current_mode = conn.flightmode
        if current_mode != "GUIDED":
            # GUIDEDモードに変更
            conn.set_mode(conn.mode_mapping().get("GUIDED"))
            time.sleep(1)

        # アーム処理
        conn.arducopter_arm()
        conn.motors_armed_wait()
        time.sleep(1)

        # 離陸コマンド送信
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,    # confirmation
            0,    # param1: Minimum pitch
            0,    # param2: Empty
            0,    # param3: Empty
            0,    # param4: Yaw angle
            0,    # param5: Latitude
            0,    # param6: Longitude
            altitude  # param7: Altitude
        )

        # コマンド受信確認
        msg = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
        if not msg or msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return "エラー: 離陸コマンドが拒否されました"

        return f"{altitude}m の高度まで離陸を開始しました。"
    finally:
        conn.close()

# モード変更
@mcp.tool()
def change_mode(mode: str) -> str:
    conn = connect_to_ardupilot()
    try:
        conn.wait_heartbeat()
        mode_id = conn.mode_mapping().get(mode.upper())
        if mode_id is None:
            return f"無効なモードです: {mode}"
        
        # モード変更コマンド送信
        conn.set_mode(mode_id)
        
        # モード変更確認 (最大5秒待機)
        start_time = time.time()
        while time.time() - start_time < 5:
            if conn.flightmode == mode.upper():
                return f"モードを {mode.upper()} に変更しました。"
            time.sleep(0.1)
        
        return f"警告: モード変更を確認できませんでした (現在のモード: {conn.flightmode})"
    finally:
        conn.close()

# ステータス確認
@mcp.tool()
def get_status() -> dict:
    conn = connect_to_ardupilot()
    try:
        conn.wait_heartbeat()
        heartbeat = conn.messages.get('HEARTBEAT')
        return {
            "armed": conn.motors_armed(),
            "mode": conn.flightmode,
            "system_status": heartbeat.system_status if heartbeat else None
        }
    finally:
        conn.close()

if __name__ == "__main__":
    print("MCPサーバーを起動します...")
    print(f"利用可能なツール: {[func.__name__ for func in [arm, disarm, takeoff, change_mode, get_status]]}")
    print("クライアントからの接続を待機中...")
    mcp.run(transport="stdio")
    print("MCPサーバーを終了します")
