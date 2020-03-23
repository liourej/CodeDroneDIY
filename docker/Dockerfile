# Initial image, starting distribution
FROM phusion/baseimage:0.10.0

# Use baseimage-docker's init system.
CMD ["/sbin/my_init"]

RUN add-apt-repository ppa:beineri/opt-qt-5.10.1-xenial

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    build-essential cmake wget \
    python-pip python-setuptools

RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - \
	&& apt-add-repository "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-6.0 main" \
	&& apt-get update \
	&& apt-get install -y clang-format-6.0 \
	&& ln -s /usr/bin/clang-format-6.0 /usr/local/bin/clang-format

RUN pip install --upgrade pip==9.0.3 && pip install -U platformio==3.5.2
RUN platformio platform install atmelavr

# Clean up APT when done.
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

