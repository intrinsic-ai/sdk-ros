# syntax = docker/dockerfile:experimental
FROM ubuntu:24.04

ENV CCACHE_DIR=/ccache

RUN apt update ; apt install -yq \
    build-essential

ENV PATH="/usr/lib/ccache:/usr/local/opt/ccache/libexec:$PATH"

RUN --mount=type=cache,target=/ccache/ ccache -s
RUN --mount=type=cache,target=/ccache/ \
    echo '#include <iostream>\n int main() { std::cout << "Test Success"; }' | g++ -x c++ - && \
    ./a.out
RUN --mount=type=cache,target=/ccache/ ccache -s
