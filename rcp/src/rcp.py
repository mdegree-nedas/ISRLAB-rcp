def parseArgs():
    return sys.argv[1:]


def main():
    yml_args = parseArgs()
    rcp = Rcp()
    for yml_arg in yml_args:

        yml_src = rcp.parser.syntax.yaml_parse_file(yml_arg)
        cfg_dict = rcp.parser.syntax.yaml_to_dict(yml_src)
        cfg_parse = rcp.parser.structure.cfg_validate(cfg_dict)

        rcp.generator.noros_core.generate(cfg_dict, cfg_parse)
        rcp.generator.noros_broker.generate(cfg_dict, cfg_parse)
        rcp.generator.noros_template.generate(cfg_dict, cfg_parse)

        rcp.generator.ros_core.generate(cfg_dict, cfg_parse)
        rcp.generator.ros_broker.generate(cfg_dict, cfg_parse)
        rcp.generator.ros_interface.generate(cfg_dict, cfg_parse)


if __name__ == "__main__":
    from librcp.core.core import Rcp
    import sys

    main()
