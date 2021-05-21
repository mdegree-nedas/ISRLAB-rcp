FROM ros:dashing

RUN apt update && \
    apt install -y python3-pip && \
    pip3 install -U argcomplete

# redis python pip
RUN ["pip3", "install", "redis==3.5.3"]

ENTRYPOINT ["/ros_entrypoint.sh"]
