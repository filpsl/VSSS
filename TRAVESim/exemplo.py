import socket

from vssproto.simulation.command_pb2 import Command, Commands
from vssproto.simulation.common_pb2 import Frame
from vssproto.simulation.packet_pb2 import Environment, Packet


def goalie_command(frame: Frame, yellowteam: bool) -> tuple[float, float]:  # noqa: ARG001, FBT001
    return 1, -1


def defender_command(frame: Frame, yellowteam: bool) -> tuple[float, float]:  # noqa: ARG001, FBT001
    return -1, 1


def attacker_command(frame: Frame, yellowteam: bool) -> tuple[float, float]:  # noqa: ARG001, FBT001
    return 1, -1


def main(yellow_team: bool) -> None:  # noqa: FBT001
    ###################################################################################
    # Connection setup
    ###################################################################################

    sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_in.bind(("224.0.0.1", 10002))

    sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_out.connect(("127.0.0.1", 20012))

    ###################################################################################
    # Loop start
    ###################################################################################

    print("Loop start")

    try:
        while True:
            ###########################################################################
            # Receive data from simulator
            ###########################################################################

            data, address = sock_in.recvfrom(1024)
            print(f"Received {len(data)} bytes from {address}")

            environment_data = Environment()
            environment_data.ParseFromString(data)

            print(environment_data.frame.ball.x)

            ###########################################################################
            # Process data from simulator
            ###########################################################################

            goalie_left_wheel, goalie_right_wheel = goalie_command(
                environment_data.frame,
                yellowteam=yellow_team,
            )

            defender_left_wheel, defender_right_wheel = defender_command(
                environment_data.frame,
                yellowteam=yellow_team,
            )

            attacker_left_wheel, attacker_right_wheel = attacker_command(
                environment_data.frame,
                yellowteam=yellow_team,
            )

            ###########################################################################
            # Send commands to simulator
            ###########################################################################

            cmd_packet = Commands()
            cmd_packet.robot_commands.append(
                Command(
                    id=0,
                    yellowteam=yellow_team,
                    wheel_left=goalie_left_wheel,
                    wheel_right=goalie_right_wheel,
                ),
            )

            cmd_packet.robot_commands.append(
                Command(
                    id=1,
                    yellowteam=yellow_team,
                    wheel_left=defender_left_wheel,
                    wheel_right=defender_right_wheel,
                ),
            )

            cmd_packet.robot_commands.append(
                Command(
                    id=2,
                    yellowteam=yellow_team,
                    wheel_left=attacker_left_wheel,
                    wheel_right=attacker_right_wheel,
                ),
            )

            packet = Packet()
            packet.cmd.CopyFrom(cmd_packet)

            sock_out.send(packet.SerializeToString())

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        sock_out.close()
        sock_in.close()


if __name__ == "__main__":
    main(yellow_team=True)
