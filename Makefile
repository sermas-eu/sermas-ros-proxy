
stop:
	docker compose down || true

start: stop
	docker compose up

build:
	docker compose build