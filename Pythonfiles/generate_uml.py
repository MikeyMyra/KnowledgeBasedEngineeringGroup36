# #!/usr/bin/env python
# import ast
# import os
# import sys
# from typing import Dict, List, Tuple, Optional, Set
#
#
# class ClassInfo:
#     def __init__(self, name: str, module: str):
#         self.name: str = name
#         self.module: str = module
#         self.bases: List[str] = []  # base class names
#         self.inputs: List[Tuple[str, Optional[str]]] = []  # (name, type)
#         self.attributes: List[str] = []  # @Attribute methods
#         self.parts: List[Tuple[str, Optional[str]]] = []  # (part_name, child_type)
#         # for later: other methods / fields could be added
#
#
# def get_full_name(node: ast.AST) -> str:
#     """Return last part of name or attribute, e.g. 'Attribute' from 'core.Attribute'."""
#     if isinstance(node, ast.Name):
#         return node.id
#     if isinstance(node, ast.Attribute):
#         return node.attr
#     return ""
#
#
# def extract_type_annotation(ann: Optional[ast.AST]) -> Optional[str]:
#     """Turn an annotation node into a simple string."""
#     if ann is None:
#         return None
#     if isinstance(ann, ast.Name):
#         return ann.id
#     if isinstance(ann, ast.Subscript):
#         # e.g. tuple[float, float] -> "tuple"
#         if isinstance(ann.value, ast.Name):
#             return ann.value.id
#         if isinstance(ann.value, ast.Attribute):
#             return ann.value.attr
#     if isinstance(ann, ast.Attribute):
#         return ann.attr
#     return None
#
#
# def is_input_call(node: ast.AST) -> bool:
#     """Check if node is a call to Input(...)."""
#     if not isinstance(node, ast.Call):
#         return False
#     func_name = get_full_name(node.func)
#     return func_name == "Input"
#
#
# def is_decorator_named(decorator: ast.AST, name: str) -> bool:
#     """Check if decorator is @name, possibly qualified."""
#     if isinstance(decorator, ast.Name):
#         return decorator.id == name
#     if isinstance(decorator, ast.Attribute):
#         return decorator.attr == name
#     return False
#
#
# def get_part_child_type(func_def: ast.FunctionDef) -> Optional[str]:
#     """Try to deduce the child type returned by a @Part method.
#
#     We handle common ParaPy pattern:
#         @Part
#         def wing(self) -> Wing:
#             return Wing(...)
#
#     First use return annotation, else inspect the return statement.
#     """
#     # 1) Use return annotation if present
#     if func_def.returns is not None:
#         ann_name = extract_type_annotation(func_def.returns)
#         if ann_name:
#             return ann_name
#
#     # 2) Inspect body for a simple 'return SomeClass(...)'
#     for stmt in func_def.body:
#         if isinstance(stmt, ast.Return) and isinstance(stmt.value, ast.Call):
#             func_name = get_full_name(stmt.value.func)
#             if func_name:
#                 return func_name
#
#     return None
#
#
# def process_class(node: ast.ClassDef, module: str) -> ClassInfo:
#     info = ClassInfo(name=node.name, module=module)
#
#     # bases
#     for b in node.bases:
#         base_name = get_full_name(b)
#         if base_name:
#             info.bases.append(base_name)
#
#     # class body: inputs (class attributes) and decorated methods
#     for stmt in node.body:
#         # Inputs: class attributes with value Input(...)
#         if isinstance(stmt, ast.AnnAssign):
#             # e.g. foo: float = Input(...)
#             target = stmt.target
#             if isinstance(target, ast.Name) and is_input_call(stmt.value):
#                 slot_name = target.id
#                 slot_type = extract_type_annotation(stmt.annotation)
#                 info.inputs.append((slot_name, slot_type))
#         elif isinstance(stmt, ast.Assign):
#             # less common but support: foo = Input(...)
#             if len(stmt.targets) == 1 and is_input_call(stmt.value):
#                 target = stmt.targets[0]
#                 if isinstance(target, ast.Name):
#                     slot_name = target.id
#                     info.inputs.append((slot_name, None))
#
#         # Methods: @Attribute, @Part
#         if isinstance(stmt, ast.FunctionDef):
#             decorators = stmt.decorator_list or []
#             for dec in decorators:
#                 if is_decorator_named(dec, "Attribute"):
#                     info.attributes.append(stmt.name)
#                 elif is_decorator_named(dec, "Part"):
#                     child_type = get_part_child_type(stmt)
#                     info.parts.append((stmt.name, child_type))
#
#     return info
#
#
# def walk_project(root_dir: str) -> Dict[str, ClassInfo]:
#     """Parse all .py files under root_dir and collect ClassInfo."""
#     classes: Dict[str, ClassInfo] = {}
#
#     for dirpath, dirnames, filenames in os.walk(root_dir):
#         for filename in filenames:
#             if not filename.endswith(".py"):
#                 continue
#             full_path = os.path.join(dirpath, filename)
#             rel_module = os.path.relpath(full_path, root_dir)
#             # convert path/to/file.py -> path.to.file (rough module name)
#             module_name = os.path.splitext(rel_module)[0].replace(os.sep, ".")
#
#             try:
#                 with open(full_path, "r", encoding="utf-8") as f:
#                     src = f.read()
#                 tree = ast.parse(src, filename=full_path)
#             except Exception as e:
#                 print(f"# Skipping {full_path}: parse error {e}", file=sys.stderr)
#                 continue
#
#             for node in ast.walk(tree):
#                 if isinstance(node, ast.ClassDef):
#                     info = process_class(node, module_name)
#                     # store by simple name; if duplicates, last one wins
#                     classes[info.name] = info
#
#     return classes
#
#
# def build_relationships(classes: Dict[str, ClassInfo]) -> Tuple[Set[Tuple[str, str]], Set[Tuple[str, str]]]:
#     """Return (inheritance, composition) sets of (parent, child)."""
#     inheritance: Set[Tuple[str, str]] = set()
#     composition: Set[Tuple[str, str]] = set()
#
#     known_class_names = set(classes.keys())
#
#     # inheritance: class A(Base1, Base2)
#     for cls in classes.values():
#         for base in cls.bases:
#             inheritance.add((base, cls.name))
#
#     # composition: for each @Part that returns a known class
#     for cls in classes.values():
#         for part_name, child_type in cls.parts:
#             if child_type and child_type in known_class_names:
#                 composition.add((cls.name, child_type))
#
#     return inheritance, composition
#
#
# def escape_puml(s: str) -> str:
#     """Simple escape if needed; currently just return as-is."""
#     return s
#
#
# def generate_plantuml(classes: Dict[str, ClassInfo]) -> str:
#     inheritance, composition = build_relationships(classes)
#
#     lines: List[str] = []
#     lines.append("@startuml")
#
#     # Classes
#     for cls in sorted(classes.values(), key=lambda c: c.name):
#         lines.append(f"class {escape_puml(cls.name)} {{")
#
#         # show module in a note-like separator
#         if cls.module:
#             lines.append(f"  .. {escape_puml(cls.module)} ..")
#
#         # Inputs
#         if cls.inputs:
#             for name, typ in cls.inputs:
#                 t = typ or "?"
#                 lines.append(f"  +{name}: {t} <<Input>>")
#
#         # Attributes (methods)
#         if cls.attributes:
#             lines.append("  .. Attributes ..")
#             for name in cls.attributes:
#                 lines.append(f"  +{name}() <<Attribute>>")
#
#         # Parts
#         if cls.parts:
#             lines.append("  .. Parts ..")
#             for name, child_type in cls.parts:
#                 suffix = f": {child_type}" if child_type else ""
#                 lines.append(f"  +{name}(){suffix} <<Part>>")
#
#         lines.append("}")
#
#     # Inheritance
#     for base, derived in sorted(inheritance):
#         lines.append(f"{escape_puml(base)} <|-- {escape_puml(derived)}")
#
#     # Composition
#     for parent, child in sorted(composition):
#         lines.append(f"{escape_puml(parent)} *-- {escape_puml(child)}")
#
#     lines.append("@enduml")
#     return "\n".join(lines)
#
#
# def main(argv: List[str]) -> int:
#     if len(argv) < 2:
#         print(f"Usage: {argv[0]} <project_root_dir>", file=sys.stderr)
#         return 1
#
#     root_dir = os.path.abspath(argv[1])
#     if not os.path.isdir(root_dir):
#         print(f"Error: {root_dir} is not a directory", file=sys.stderr)
#         return 1
#
#     classes = walk_project(root_dir)
#     puml = generate_plantuml(classes)
#     print(puml)
#     return 0
#
#
# if __name__ == "__main__":
#     raise SystemExit(main(sys.argv))

#!/usr/bin/env python
import ast
import os
import sys
from typing import Dict, List, Tuple, Optional, Set


class ClassInfo:
    def __init__(self, name: str, module: str):
        self.name: str = name
        self.module: str = module
        self.bases: List[str] = []  # base class names
        self.inputs: List[Tuple[str, Optional[str]]] = []  # (name, type)
        self.attributes: List[str] = []  # @Attribute methods
        self.parts: List[Tuple[str, Optional[str]]] = []  # (part_name, child_type)


def get_full_name(node: ast.AST) -> str:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return node.attr
    return ""


def extract_type_annotation(ann: Optional[ast.AST]) -> Optional[str]:
    if ann is None:
        return None
    if isinstance(ann, ast.Name):
        return ann.id
    if isinstance(ann, ast.Subscript):
        if isinstance(ann.value, ast.Name):
            return ann.value.id
        if isinstance(ann.value, ast.Attribute):
            return ann.value.attr
    if isinstance(ann, ast.Attribute):
        return ann.attr
    return None


def is_input_call(node: ast.AST) -> bool:
    if not isinstance(node, ast.Call):
        return False
    return get_full_name(node.func) == "Input"


def is_decorator_named(decorator: ast.AST, name: str) -> bool:
    if isinstance(decorator, ast.Name):
        return decorator.id == name
    if isinstance(decorator, ast.Attribute):
        return decorator.attr == name
    return False


def get_part_child_type(func_def: ast.FunctionDef) -> Optional[str]:
    if func_def.returns is not None:
        ann_name = extract_type_annotation(func_def.returns)
        if ann_name:
            return ann_name
    for stmt in func_def.body:
        if isinstance(stmt, ast.Return) and isinstance(stmt.value, ast.Call):
            func_name = get_full_name(stmt.value.func)
            if func_name:
                return func_name
    return None


def process_class(node: ast.ClassDef, module: str) -> ClassInfo:
    info = ClassInfo(name=node.name, module=module)

    for b in node.bases:
        base_name = get_full_name(b)
        if base_name:
            info.bases.append(base_name)

    for stmt in node.body:
        if isinstance(stmt, ast.AnnAssign):
            target = stmt.target
            if isinstance(target, ast.Name) and is_input_call(stmt.value):
                info.inputs.append((target.id, extract_type_annotation(stmt.annotation)))
        elif isinstance(stmt, ast.Assign):
            if len(stmt.targets) == 1 and is_input_call(stmt.value):
                target = stmt.targets[0]
                if isinstance(target, ast.Name):
                    info.inputs.append((target.id, None))

        if isinstance(stmt, ast.FunctionDef):
            for dec in (stmt.decorator_list or []):
                if is_decorator_named(dec, "Attribute"):
                    info.attributes.append(stmt.name)
                elif is_decorator_named(dec, "Part"):
                    info.parts.append((stmt.name, get_part_child_type(stmt)))

    return info


def walk_project(root_dir: str) -> Dict[str, ClassInfo]:
    classes: Dict[str, ClassInfo] = {}

    for dirpath, dirnames, filenames in os.walk(root_dir):
        for filename in filenames:
            if not filename.endswith(".py"):
                continue
            full_path = os.path.join(dirpath, filename)
            rel_module = os.path.relpath(full_path, root_dir)
            module_name = os.path.splitext(rel_module)[0].replace(os.sep, ".")

            try:
                with open(full_path, "r", encoding="utf-8") as f:
                    src = f.read()
                tree = ast.parse(src, filename=full_path)
            except Exception as e:
                print(f"# Skipping {full_path}: parse error {e}", file=sys.stderr)
                continue

            for node in ast.walk(tree):
                if isinstance(node, ast.ClassDef):
                    classes[node.name] = process_class(node, module_name)

    return classes


def build_relationships(classes: Dict[str, ClassInfo]) -> Tuple[Set[Tuple[str, str]], Set[Tuple[str, str]]]:
    inheritance: Set[Tuple[str, str]] = set()
    composition: Set[Tuple[str, str]] = set()
    known = set(classes.keys())

    for cls in classes.values():
        for base in cls.bases:
            inheritance.add((base, cls.name))

    for cls in classes.values():
        for _, child_type in cls.parts:
            if child_type and child_type in known:
                composition.add((cls.name, child_type))

    return inheritance, composition


def escape_puml(s: str) -> str:
    return s


def generate_plantuml(classes: Dict[str, ClassInfo]) -> str:
    inheritance, composition = build_relationships(classes)

    # Child types already shown via composition arrows — skip redundant type
    # suffixes on @Part members to cut down repeated text.
    composed_children: Set[str] = {child for _, child in composition}

    lines: List[str] = []
    lines.append("@startuml")
    lines.append("hide empty members")
    lines.append("skinparam classAttributeIconSize 0")

    for cls in sorted(classes.values(), key=lambda c: c.name):
        members: List[str] = []

        # Inputs — shorten stereotype to <<I>>
        for name, typ in cls.inputs:
            t = f":{typ}" if typ else ""
            members.append(f" +{name}{t} <<I>>")

        # Attributes — shorten stereotype to <<A>>
        for name in cls.attributes:
            members.append(f" +{name}() <<A>>")

        # Parts — shorten stereotype to <<P>>, omit type when arrow covers it
        for name, child_type in cls.parts:
            suffix = (f":{child_type}"
                      if child_type and child_type not in composed_children
                      else "")
            members.append(f" +{name}(){suffix} <<P>>")

        if members:
            lines.append(f"class {escape_puml(cls.name)} {{")
            lines.extend(members)
            lines.append("}")
        else:
            # One-liner for empty classes saves two lines each
            lines.append(f"class {escape_puml(cls.name)}")

    for base, derived in sorted(inheritance):
        lines.append(f"{escape_puml(base)} <|-- {escape_puml(derived)}")

    for parent, child in sorted(composition):
        lines.append(f"{escape_puml(parent)} *-- {escape_puml(child)}")

    lines.append("@enduml")
    return "\n".join(lines)


def main(argv: List[str]) -> int:
    if len(argv) < 2:
        print(f"Usage: {argv[0]} <project_root_dir>", file=sys.stderr)
        return 1

    root_dir = os.path.abspath(argv[1])
    if not os.path.isdir(root_dir):
        print(f"Error: {root_dir} is not a directory", file=sys.stderr)
        return 1

    classes = walk_project(root_dir)
    puml = generate_plantuml(classes)
    print(puml)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))