FROM rust:alpine as builder
WORKDIR /app
RUN apk add --no-cache libusb-dev
COPY . .
RUN cargo install --path .

FROM alpine:latest
COPY --from=builder /usr/local/cargo/bin/antdump /usr/local/bin/antdump
CMD ["antdump"]