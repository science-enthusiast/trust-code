import pathlib
import textwrap
import datetime
from trustpy.trad2_utilities import TRAD2Content, TRAD2Block, TRAD2Attr, TRAD2BlockList


################################################################
# LOGGER
################################################################

def init_logger():
    import logging
    logger = logging.getLogger("trad2_pydantic")
    handler = logging.StreamHandler()
    formatter = logging.Formatter(fmt="%(asctime)s %(levelname)-8s %(message)s", datefmt="%H:%M:%S")
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    return logger
logger = init_logger()


################################################################



def valid_variable_name(s):
    """ Make a valid variable name from any str. """

    import re
    import keyword

    # replace invalid characters with underscores
    s = re.sub(r'[^0-9a-zA-Z_]', '_', s)

    # prefix leading number with 'i'
    s = re.sub(r'^([0-9])', r'i\1', s)

    # suffix reserved keywords with '_'
    if keyword.iskeyword(s):
        s += '_'

    return s


def change_class_name(s):
    return s.capitalize()


def write_block(block, file, all_blocks):
    """ Write a TRAD2Block as pydantic class. """

    assert isinstance(block, TRAD2Block)

    if block.written:
        return

    logger.debug(block.nam)

    # dependencies must be written before self
    dependencies = [block.name_base] + [a.typ for a in block.attrs]
    for dependency in dependencies:
        dependency = all_blocks.get(dependency, None)
        if dependency:
            write_block(dependency, file, all_blocks)

    lines = [
        f'#' * 64,
        f'',
        f'class {change_class_name(block.nam)}({change_class_name(block.name_base) or "BaseModel"}):',
        f'    __braces:int = {block.mode}',
        f'    __description: str = r"{block.desc}"',
        f'    __synonyms: str = {block.synos}',
    ]

    for attribute in block.attrs:

        assert isinstance(attribute, TRAD2Attr)

        attr_name = attribute.nam
        attr_type = attribute.typ
        attr_syno = attribute.synos
        attr_mode = attribute.is_opt
        attr_desc = attribute.desc

        attr_name = valid_variable_name(attr_name)

        args = f'default_factory=lambda: eval("{change_class_name(attr_type)}()")'

        if attr_type in all_blocks:
            cls = all_blocks[attr_type]
            if isinstance(cls, TRAD2BlockList):
                attr_type = f'Annotated[List["{change_class_name(cls.classtype)}"], {cls.comma}]'
                attr_desc = cls.desc
                args = f'default=[]'
            else:
                attr_type = change_class_name(attr_type)

        elif attr_type == "entier":
            attr_type = "int"
            args = f'default=0'

        elif attr_type == "floattant":
            attr_type = "float"
            args = f'default=0.0'

        elif attr_type == "chaine":
            # TODO create custom class str_trust to handle "{ how do you { do } }" ?
            # SEE class chaine_Tru
            attr_type = "str"
            args = f'default=""'

        elif attr_type in ["list", "listf"]:
            # TODO handle difference list vs listf ?
            attr_type = "List[float]"
            args = "default_factory=list"

        elif attr_type in ["listentier", "listentierf"]:
            # TODO handle difference listentier vs listentierf ?
            attr_type = "List[int]"
            args = "default_factory=list"

        elif attr_type in ["listchaine", "listchainef"]:
            # TODO handle difference listchaine vs listchainef ?
            attr_type = "List[str]"
            args = "default_factory=list"

        elif attr_type.startswith("chaine(into="):
            choices = attr_type[13:].split("]")[0]
            attr_type= f'Literal[{choices}]'
            args = f'default={choices.split(",")[0]}'

        elif attr_type.startswith("entier(into="):
            choices = attr_type[13:].split("]")[0]
            choices = [int(x.strip('"').strip("'")) for x in choices.split(",")]
            attr_type= f'Literal{choices}'
            args = f'default={choices[0]}'

        elif attr_type.startswith("entier(max="):
            max_val = int(attr_type[11:-1])
            attr_type= "int"
            args = f'default=0, le={max_val}'

        elif attr_type == "rien":
            attr_type = "bool"
            args = f'default=False'

        elif attr_type == "suppress_param":
            # NOTE tricky
            # hide the inherited *instance* attribute by a *class* attribute with same name
            attr_type = "ClassVar[str]"
            attr_desc = "suppress_param"
            args = 'default="suppress_param"'

        elif attr_type.startswith("ref_"):
            attr_type = "str"
            args = f'default=""'

        else:
            message = f"unresolved type : {attr_type}"
            logger.error(message)
            raise NotImplementedError(message)

        lines.append(f'    {attr_name}: {attr_type} = Field(description=r"{attr_desc}", {args})')

    lines.append('\n')

    # actual file writing
    file.write('\n'.join(lines))

    # mark this block as written
    block.written = True


def generate_pydantic(trad2_filename, output_filename, testing=False):

    trad2_filename = pathlib.Path(trad2_filename)

    all_blocks = TRAD2Content.BuildContentFromTRAD2(trad2_filename).data

    # add two base classes that are required and not declared in TRAD2 file
    objet_u = TRAD2Block()
    objet_u.nam = "objet_u"
    objet_u.mode = 1
    listobj_impl = TRAD2Block()
    listobj_impl.nam = "listobj_impl"
    listobj_impl.mode = 1
    all_blocks += [objet_u, listobj_impl]

    # make a dict to easily find block by name
    all_blocks = { block.nam: block for block in all_blocks }

    # add two properties used during writing of blocks
    for block in all_blocks.values():
        block.written = False

    header = f'''
        ################################################################
        # This file was generated automatically from :
        # {str(trad2_filename.resolve())}
        # {datetime.datetime.now().strftime(r"%y-%m-%d at %H:%M:%S")}
        ################################################################

        import sys        
        from typing_extensions import Annotated, Literal
        from typing import ClassVar, List
        from pydantic import BaseModel, Field
    '''

    if output_filename is None:
        output_filename = trad2_filename.name + ".py"

    with open(output_filename, "w", encoding="utf-8") as file:
        file.write(textwrap.dedent(header).strip())
        file.write("\n" * 3)
        for block in all_blocks.values():
            write_block(block, file, all_blocks)

    if testing:
        test_filename = output_filename.replace(".py", "_test.py")
        with open(test_filename, "w", encoding="utf-8") as file:
            file.write(f'import {trad2_filename.name}')
            file.write("\n" * 3)
            for cls in all_blocks.keys():
                file.write(f'{trad2_filename.name}.{cls}()\n')


if __name__ == '__main__':

    trad2_filename = pathlib.Path("src/trustpy/test/TRAD_2_GENEPI_V17")
    output_filename = trad2_filename.name + ".py"
    generate_pydantic(trad2_filename, output_filename, testing=True)
