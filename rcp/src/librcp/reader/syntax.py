import yaml


class Syntax:
    def _yaml_read_file(self, yaml_src):
        with open(yaml_src, "r") as yaml_file:
            return yaml_file.read()

    def yaml_parse_file(self, yml_src):
        return self._yaml_read_file(yml_src)

    def yaml_to_dict(self, yaml_src):
        return yaml.load(yaml_src, Loader=yaml.FullLoader)
