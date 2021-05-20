from librcp.reader.parser import Parser
from librcp.generator.generator import Generator


class Rcp:
    def __init__(self):
        self.parser = Parser()
        self.generator = Generator()
