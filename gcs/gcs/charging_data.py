LISTEN_PORT = 12345


def receive_udp_messages() -> None:
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.bind(("", LISTEN_PORT))

	print(f"Listening on port {LISTEN_PORT}")
	
	try:
		while True:
			data, addr = sock.recvfrom(4096)
			sender_ip, sender_port = addr
			message = data.decode("utf-8", errors="replace")

			print(f"From {sender_ip}:{sender_port}")
			print(message)
			print("-" * 40)
	except KeyboardInterrupt:
		print("Stopped by user.")
	finally:
		sock.close()


if _name_ == "_main_":
	receive_udp_messages()