
stop:
	docker compose down || true

start: stop
	docker compose up

