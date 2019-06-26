FROM ubuntu:18.04

# Install dependencies
RUN apt-get update -y && apt-get install -y libgtest-dev cmake gcc-8 g++-8 libasan5 google-mock git
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8

# Compile GTest
WORKDIR /usr/src/gtest
RUN cmake CMakeLists.txt && make -j$(nproc)
RUN cp *.a /usr/lib
WORKDIR /usr/src/gmock
RUN cmake CMakeLists.txt && make -j$(nproc)
RUN cp *.a /usr/lib

# Compile Messages
WORKDIR /
RUN git clone https://github.com/SoPra-Team-10/Messages.git
WORKDIR /Messages
RUN cmake . && make -j$(nproc) SopraMessages && make install

# Compile GameLogic
WORKDIR /
RUN git clone https://github.com/SoPra-Team-10/GameLogic.git
WORKDIR /GameLogic
RUN cmake . && make -j$(nproc) SopraGameLogic && make install

RUN ldconfig

WORKDIR /
RUN mkdir /src
COPY . /src/
RUN rm -rf /src/build
RUN mkdir -p /src/build

WORKDIR /src/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. && make -j$(nproc) SopraAITools && make install
RUN cmake -DCMAKE_BUILD_TYPE=Release -DUSE_INSTALLED_LIB=true .. && make -j$(nproc) Tests

WORKDIR /src
CMD ["build/Tests/Tests", "--gtest_repeat=10", "--gtest_shuffle", "--gtest_color=yes"]
