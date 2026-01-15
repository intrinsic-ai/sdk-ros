# syntax = docker/dockerfile:experimental
FROM php:7.3-fpm-stretch

# Create app directory
WORKDIR /usr/src/app

# setup locales
RUN apt update && \
  apt install -y locales && \
  sed -i -e 's/# en_GB.UTF-8 UTF-8/en_GB.UTF-8 UTF-8/' /etc/locale.gen && \
  locale-gen

ENV LANGUAGE=en_GB.UTF-8
ENV LANG=en_GB.UTF-8
ENV LC_ALL=en_GB.UTF-8
ENV CCACHE_DIR=/ccache

RUN apt update ; apt install -yq \
        git \
        cloud-guest-utils \
        iproute2 \
        libxml2-dev \
        libxslt1-dev \
        libmemcached-dev \
        libgd-dev \
        libzip-dev \
        libmemcached-dev \
        ccache \
        awscli


# use ccache (make it appear in path earlier then /usr/bin/gcc etc)
RUN for p in gcc g++ cc c++; do ln -vs /usr/bin/ccache /usr/local/bin/$p;  done

# prod format
RUN --mount=type=cache,target=/ccache/ docker-php-source extract && \
    docker-php-ext-install \
    intl \
    bcmath  \
    calendar \
    exif \
    gd \
    gettext \
    mysqli \
    opcache \
    pcntl \
    pdo_mysql \
    shmop \
    soap \
    sockets \
    sysvmsg \
    sysvsem \
    sysvshm \
    xsl \
    zip \
  && \
  docker-php-source delete
RUN --mount=type=cache,target=/ccache/ ccache -s
