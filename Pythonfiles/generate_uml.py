#-------------------------------
#version 1
#-------------------------------

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


#-------------------------------
#version 2
#-------------------------------

import ast
import os
import sys
from typing import Dict, List, Tuple, Optional, Set


# class ClassInfo:
#     def __init__(self, name: str, module: str):
#         self.name: str = name
#         self.module: str = module
#         self.bases: List[str] = []  # base class names
#         self.inputs: List[Tuple[str, Optional[str]]] = []  # (name, type)
#         self.attributes: List[str] = []  # @Attribute methods
#         self.parts: List[Tuple[str, Optional[str]]] = []  # (part_name, child_type)
#
#
# def get_full_name(node: ast.AST) -> str:
#     if isinstance(node, ast.Name):
#         return node.id
#     if isinstance(node, ast.Attribute):
#         return node.attr
#     return ""
#
#
# def extract_type_annotation(ann: Optional[ast.AST]) -> Optional[str]:
#     if ann is None:
#         return None
#     if isinstance(ann, ast.Name):
#         return ann.id
#     if isinstance(ann, ast.Subscript):
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
#     if not isinstance(node, ast.Call):
#         return False
#     return get_full_name(node.func) == "Input"
#
#
# def is_decorator_named(decorator: ast.AST, name: str) -> bool:
#     if isinstance(decorator, ast.Name):
#         return decorator.id == name
#     if isinstance(decorator, ast.Attribute):
#         return decorator.attr == name
#     return False
#
#
# def get_part_child_type(func_def: ast.FunctionDef) -> Optional[str]:
#     if func_def.returns is not None:
#         ann_name = extract_type_annotation(func_def.returns)
#         if ann_name:
#             return ann_name
#     for stmt in func_def.body:
#         if isinstance(stmt, ast.Return) and isinstance(stmt.value, ast.Call):
#             func_name = get_full_name(stmt.value.func)
#             if func_name:
#                 return func_name
#     return None
#
#
# def process_class(node: ast.ClassDef, module: str) -> ClassInfo:
#     info = ClassInfo(name=node.name, module=module)
#
#     for b in node.bases:
#         base_name = get_full_name(b)
#         if base_name:
#             info.bases.append(base_name)
#
#     for stmt in node.body:
#         if isinstance(stmt, ast.AnnAssign):
#             target = stmt.target
#             if isinstance(target, ast.Name) and is_input_call(stmt.value):
#                 info.inputs.append((target.id, extract_type_annotation(stmt.annotation)))
#         elif isinstance(stmt, ast.Assign):
#             if len(stmt.targets) == 1 and is_input_call(stmt.value):
#                 target = stmt.targets[0]
#                 if isinstance(target, ast.Name):
#                     info.inputs.append((target.id, None))
#
#         if isinstance(stmt, ast.FunctionDef):
#             for dec in (stmt.decorator_list or []):
#                 if is_decorator_named(dec, "Attribute"):
#                     info.attributes.append(stmt.name)
#                 elif is_decorator_named(dec, "Part"):
#                     info.parts.append((stmt.name, get_part_child_type(stmt)))
#
#     return info
#
#
# def walk_project(root_dir: str) -> Dict[str, ClassInfo]:
#     classes: Dict[str, ClassInfo] = {}
#
#     for dirpath, dirnames, filenames in os.walk(root_dir):
#         for filename in filenames:
#             if not filename.endswith(".py"):
#                 continue
#             full_path = os.path.join(dirpath, filename)
#             rel_module = os.path.relpath(full_path, root_dir)
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
#                     classes[node.name] = process_class(node, module_name)
#
#     return classes
#
#
# def build_relationships(classes: Dict[str, ClassInfo]) -> Tuple[Set[Tuple[str, str]], Set[Tuple[str, str]]]:
#     inheritance: Set[Tuple[str, str]] = set()
#     composition: Set[Tuple[str, str]] = set()
#     known = set(classes.keys())
#
#     for cls in classes.values():
#         for base in cls.bases:
#             inheritance.add((base, cls.name))
#
#     for cls in classes.values():
#         for _, child_type in cls.parts:
#             if child_type and child_type in known:
#                 composition.add((cls.name, child_type))
#
#     return inheritance, composition
#
#
# def escape_puml(s: str) -> str:
#     return s
#
#
# def generate_plantuml(classes: Dict[str, ClassInfo]) -> str:
#     inheritance, composition = build_relationships(classes)
#
#     # Child types already shown via composition arrows — skip redundant type
#     # suffixes on @Part members to cut down repeated text.
#     composed_children: Set[str] = {child for _, child in composition}
#
#     lines: List[str] = []
#     lines.append("@startuml")
#     lines.append("hide empty members")
#     lines.append("skinparam classAttributeIconSize 0")
#
#     for cls in sorted(classes.values(), key=lambda c: c.name):
#         members: List[str] = []
#
#         # Inputs — shorten stereotype to <<I>>
#         for name, typ in cls.inputs:
#             t = f":{typ}" if typ else ""
#             members.append(f" +{name}{t} <<I>>")
#
#         # Attributes — shorten stereotype to <<A>>
#         for name in cls.attributes:
#             members.append(f" +{name}() <<A>>")
#
#         # Parts — shorten stereotype to <<P>>, omit type when arrow covers it
#         for name, child_type in cls.parts:
#             suffix = (f":{child_type}"
#                       if child_type and child_type not in composed_children
#                       else "")
#             members.append(f" +{name}(){suffix} <<P>>")
#
#         if members:
#             lines.append(f"class {escape_puml(cls.name)} {{")
#             lines.extend(members)
#             lines.append("}")
#         else:
#             # One-liner for empty classes saves two lines each
#             lines.append(f"class {escape_puml(cls.name)}")
#
#     for base, derived in sorted(inheritance):
#         lines.append(f"{escape_puml(base)} <|-- {escape_puml(derived)}")
#
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


#-------------------------------
#version 3
#-------------------------------

import ast
import os
import sys
from typing import Dict, List, Tuple, Optional, Set


class ClassInfo:
    def __init__(self, name: str, module: str):
        self.name: str = name
        self.module: str = module
        self.bases: List[str] = []
        self.inputs: List[Tuple[str, Optional[str]]] = []
        self.attributes: List[str] = []
        self.parts: List["PartInfo"] = []


class PartInfo:
    def __init__(self, name: str, child_types: List[str], multiplicity: Optional[Tuple[int, Optional[int]]]):
        self.name: str = name
        # All possible return types (one per branch if dynamic)
        self.child_types: List[str] = child_types
        # (min, max) where max=None means unbounded (*)
        self.multiplicity: Optional[Tuple[int, Optional[int]]] = multiplicity


# ---------------------------------------------------------------------------
# AST helpers
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Collect all Return-call class names from a function body, including inside
# if/else branches, to support dynamic @Part types.
# ---------------------------------------------------------------------------

def collect_return_types(func_def: ast.FunctionDef) -> List[str]:
    """Walk all Return nodes in the function and collect called class names."""
    types: List[str] = []
    for node in ast.walk(func_def):
        if isinstance(node, ast.Return) and node.value is not None:
            call = node.value
            if isinstance(call, ast.Call):
                name = get_full_name(call.func)
                if name and name not in types:
                    types.append(name)
            elif isinstance(call, ast.IfExp):
                # ternary: A(...) if cond else B(...)
                for branch in (call.body, call.orelse):
                    if isinstance(branch, ast.Call):
                        name = get_full_name(branch.func)
                        if name and name not in types:
                            types.append(name)
    return types


def get_part_child_types(func_def: ast.FunctionDef) -> List[str]:
    """Return all possible child class names for a @Part method."""
    # 1) Return annotation — single static type
    if func_def.returns is not None:
        ann_name = extract_type_annotation(func_def.returns)
        if ann_name:
            return [ann_name]
    # 2) Inspect body for return statements (may be multiple branches)
    return collect_return_types(func_def)


# ---------------------------------------------------------------------------
# Parse quantity_in_range keyword from a @Part decorator call.
#
# ParaPy supports:
#   @Part(quantity_in_range=(1, 3))
#   @Part(quantity_in_range=(1, None))   <- unbounded
# ---------------------------------------------------------------------------

def parse_quantity_in_range(decorator: ast.AST) -> Optional[Tuple[int, Optional[int]]]:
    if not isinstance(decorator, ast.Call):
        return None
    for kw in decorator.keywords:
        if kw.arg != "quantity_in_range":
            continue
        val = kw.value
        if isinstance(val, (ast.Tuple, ast.List)) and len(val.elts) == 2:
            lo_node, hi_node = val.elts
            lo = lo_node.value if isinstance(lo_node, ast.Constant) else None
            if isinstance(hi_node, ast.Constant) and hi_node.value is None:
                hi = None
            elif isinstance(hi_node, ast.Constant):
                hi = int(hi_node.value)
            else:
                hi = None
            if lo is not None:
                return (int(lo), hi)
    return None


# ---------------------------------------------------------------------------
# Class processing
# ---------------------------------------------------------------------------

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
                    child_types = get_part_child_types(stmt)
                    multiplicity = parse_quantity_in_range(dec)
                    info.parts.append(PartInfo(stmt.name, child_types, multiplicity))

    return info


def walk_project(root_dir: str) -> Dict[str, ClassInfo]:
    classes: Dict[str, ClassInfo] = {}

    for dirpath, _, filenames in os.walk(root_dir):
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


# ---------------------------------------------------------------------------
# Filtering: keep GeomBase itself and every class that transitively inherits
# from it (downward through the inheritance tree only).
# ---------------------------------------------------------------------------

def find_geombase_subclasses(classes: Dict[str, ClassInfo], root: str = "GeomBase") -> Set[str]:
    kept: Set[str] = set()

    # Build downward map: parent -> set of direct children
    children_of: Dict[str, Set[str]] = {name: set() for name in classes}
    for cls in classes.values():
        for base in cls.bases:
            if base in children_of:
                children_of[base].add(cls.name)

    if root not in classes:
        print(f"# Warning: '{root}' not found in parsed classes.", file=sys.stderr)
        return set(classes.keys())

    stack = [root]
    while stack:
        name = stack.pop()
        if name in kept:
            continue
        kept.add(name)
        for child in children_of.get(name, []):
            stack.append(child)

    return kept


# ---------------------------------------------------------------------------
# Relationships
# ---------------------------------------------------------------------------

def build_relationships(
    classes: Dict[str, ClassInfo],
    kept: Set[str],
) -> Tuple[
    Set[Tuple[str, str]],
    List[Tuple[str, str, str, Optional[Tuple[int, Optional[int]]], bool]],
]:
    """
    Returns:
      inheritance  : set of (base, derived) where both are in `kept`
      compositions : list of (parent, child_type, part_name, multiplicity, is_dynamic)
                     is_dynamic=True when the @Part has >1 possible return type
    """
    inheritance: Set[Tuple[str, str]] = set()
    compositions: List[Tuple[str, str, str, Optional[Tuple[int, Optional[int]]], bool]] = []

    for cls in classes.values():
        if cls.name not in kept:
            continue
        for base in cls.bases:
            if base in kept:
                inheritance.add((base, cls.name))

    for cls in classes.values():
        if cls.name not in kept:
            continue
        for part in cls.parts:
            is_dynamic = len(part.child_types) > 1
            for child_type in part.child_types:
                if child_type in kept:
                    compositions.append(
                        (cls.name, child_type, part.name, part.multiplicity, is_dynamic)
                    )

    return inheritance, compositions


# ---------------------------------------------------------------------------
# PlantUML generation
# ---------------------------------------------------------------------------

def multiplicity_label(mult: Optional[Tuple[int, Optional[int]]]) -> str:
    if mult is None:
        return "1"
    lo, hi = mult
    if hi is None:
        return f"{lo}..*"
    if lo == hi:
        return str(lo)
    return f"{lo}..{hi}"


def escape_puml(s: str) -> str:
    return s


def generate_plantuml(classes: Dict[str, ClassInfo], root: str = "GeomBase") -> str:
    kept = find_geombase_subclasses(classes, root)
    inheritance, compositions = build_relationships(classes, kept)

    lines: List[str] = []
    lines.append("@startuml")
    lines.append("hide empty members")
    lines.append("skinparam classAttributeIconSize 0")
    lines.append("")

    # Emit only kept classes
    for cls in sorted((c for c in classes.values() if c.name in kept), key=lambda c: c.name):
        members: List[str] = []

        if cls.inputs:
            for name, typ in cls.inputs:
                t = f": {typ}" if typ else ""
                members.append(f"  +{name}{t} <<Input>>")

        if cls.attributes:
            members.append("  .. Attributes ..")
            for name in cls.attributes:
                members.append(f"  +{name}() <<Attribute>>")

        if cls.parts:
            members.append("  .. Parts ..")
            for part in cls.parts:
                # Show dynamic options as TypeA|TypeB in the member listing
                type_label = "|".join(part.child_types) if part.child_types else ""
                suffix = f": {type_label}" if type_label else ""
                members.append(f"  +{part.name}(){suffix} <<Part>>")

        if members:
            lines.append(f"class {escape_puml(cls.name)} {{")
            lines.extend(members)
            lines.append("}")
        else:
            lines.append(f"class {escape_puml(cls.name)}")

    lines.append("")

    # Inheritance arrows
    for base, derived in sorted(inheritance):
        lines.append(f"{escape_puml(base)} <|-- {escape_puml(derived)}")

    lines.append("")

    # Composition arrows.
    # Static parts  : filled diamond  *--  "mult"  Child : partName
    # Dynamic parts : dashed arrow    ..>  "mult"  Child : partName  (one arrow per possible type)
    seen_pairs: Set[Tuple[str, str]] = set()
    for parent, child, part_name, mult, is_dynamic in sorted(
        compositions, key=lambda x: (x[0], x[1], x[2])
    ):
        pair = (parent, child)
        if pair in seen_pairs:
            continue
        seen_pairs.add(pair)

        mult_str = multiplicity_label(mult)
        if is_dynamic:
            # Dashed arrow signals "one of these options is selected at runtime"
            lines.append(
                f'{escape_puml(parent)} ..> "{mult_str}" {escape_puml(child)} : {part_name} (dynamic)'
            )
        else:
            lines.append(
                f'{escape_puml(parent)} *-- "{mult_str}" {escape_puml(child)} : {part_name}'
            )

    lines.append("")
    lines.append("@enduml")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(argv: List[str]) -> int:
    if len(argv) < 2:
        print(f"Usage: {argv[0]} <project_root_dir> [RootClass=GeomBase]", file=sys.stderr)
        return 1

    root_dir = os.path.abspath(argv[1])
    if not os.path.isdir(root_dir):
        print(f"Error: {root_dir} is not a directory", file=sys.stderr)
        return 1

    root_class = argv[2] if len(argv) >= 3 else "GeomBase"

    classes = walk_project(root_dir)
    puml = generate_plantuml(classes, root=root_class)
    print(puml)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
#
# # !/usr/bin/env python

#--------------------------------
# Version 4
#--------------------------------


#!/usr/bin/env python
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
#         self.bases: List[str] = []
#         self.inputs: List[Tuple[str, Optional[str]]] = []
#         self.attributes: List[str] = []
#         self.parts: List["PartInfo"] = []
#
#
# class PartInfo:
#     def __init__(self, name: str, child_types: List[str], multiplicity: Optional[Tuple[int, Optional[int]]]):
#         self.name: str = name
#         # All possible return types (one per branch if dynamic)
#         self.child_types: List[str] = child_types
#         # (min, max) where max=None means unbounded (*)
#         self.multiplicity: Optional[Tuple[int, Optional[int]]] = multiplicity
#
#
# # ---------------------------------------------------------------------------
# # AST helpers
# # ---------------------------------------------------------------------------
#
# def get_full_name(node: ast.AST) -> str:
#     if isinstance(node, ast.Name):
#         return node.id
#     if isinstance(node, ast.Attribute):
#         return node.attr
#     return ""
#
#
# def extract_type_annotation(ann: Optional[ast.AST]) -> Optional[str]:
#     if ann is None:
#         return None
#     if isinstance(ann, ast.Name):
#         return ann.id
#     if isinstance(ann, ast.Subscript):
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
#     if not isinstance(node, ast.Call):
#         return False
#     return get_full_name(node.func) == "Input"
#
#
# def is_decorator_named(decorator: ast.AST, name: str) -> bool:
#     if isinstance(decorator, ast.Name):
#         return decorator.id == name
#     if isinstance(decorator, ast.Attribute):
#         return decorator.attr == name
#     return False
#
#
# # ---------------------------------------------------------------------------
# # Collect all Return-call class names from a function body, including inside
# # if/else branches, to support dynamic @Part types.
# # ---------------------------------------------------------------------------
#
# def collect_return_types(func_def: ast.FunctionDef) -> List[str]:
#     """Walk all Return nodes in the function and collect called class names."""
#     types: List[str] = []
#     for node in ast.walk(func_def):
#         if isinstance(node, ast.Return) and node.value is not None:
#             call = node.value
#             if isinstance(call, ast.Call):
#                 name = get_full_name(call.func)
#                 if name and name not in types:
#                     types.append(name)
#             elif isinstance(call, ast.IfExp):
#                 # ternary: A(...) if cond else B(...)
#                 for branch in (call.body, call.orelse):
#                     if isinstance(branch, ast.Call):
#                         name = get_full_name(branch.func)
#                         if name and name not in types:
#                             types.append(name)
#     return types
#
#
# def get_part_child_types(func_def: ast.FunctionDef) -> List[str]:
#     """Return all possible child class names for a @Part method."""
#     # 1) Return annotation — single static type
#     if func_def.returns is not None:
#         ann_name = extract_type_annotation(func_def.returns)
#         if ann_name:
#             return [ann_name]
#     # 2) Inspect body for return statements (may be multiple branches)
#     return collect_return_types(func_def)
#
#
# # ---------------------------------------------------------------------------
# # Parse quantity_in_range keyword from a @Part decorator call.
# #
# # ParaPy supports:
# #   @Part(quantity_in_range=(1, 3))
# #   @Part(quantity_in_range=(1, None))   <- unbounded
# # ---------------------------------------------------------------------------
#
# def parse_quantity_in_range(decorator: ast.AST) -> Optional[Tuple[int, Optional[int]]]:
#     if not isinstance(decorator, ast.Call):
#         return None
#     for kw in decorator.keywords:
#         if kw.arg != "quantity_in_range":
#             continue
#         val = kw.value
#         if isinstance(val, (ast.Tuple, ast.List)) and len(val.elts) == 2:
#             lo_node, hi_node = val.elts
#             lo = lo_node.value if isinstance(lo_node, ast.Constant) else None
#             if isinstance(hi_node, ast.Constant) and hi_node.value is None:
#                 hi = None
#             elif isinstance(hi_node, ast.Constant):
#                 hi = int(hi_node.value)
#             else:
#                 hi = None
#             if lo is not None:
#                 return (int(lo), hi)
#     return None
#
#
# # ---------------------------------------------------------------------------
# # Class processing
# # ---------------------------------------------------------------------------
#
# def process_class(node: ast.ClassDef, module: str) -> ClassInfo:
#     info = ClassInfo(name=node.name, module=module)
#
#     for b in node.bases:
#         base_name = get_full_name(b)
#         if base_name:
#             info.bases.append(base_name)
#
#     for stmt in node.body:
#         if isinstance(stmt, ast.AnnAssign):
#             target = stmt.target
#             if isinstance(target, ast.Name) and is_input_call(stmt.value):
#                 info.inputs.append((target.id, extract_type_annotation(stmt.annotation)))
#         elif isinstance(stmt, ast.Assign):
#             if len(stmt.targets) == 1 and is_input_call(stmt.value):
#                 target = stmt.targets[0]
#                 if isinstance(target, ast.Name):
#                     info.inputs.append((target.id, None))
#
#         if isinstance(stmt, ast.FunctionDef):
#             for dec in (stmt.decorator_list or []):
#                 if is_decorator_named(dec, "Attribute"):
#                     info.attributes.append(stmt.name)
#                 elif is_decorator_named(dec, "Part"):
#                     child_types = get_part_child_types(stmt)
#                     multiplicity = parse_quantity_in_range(dec)
#                     info.parts.append(PartInfo(stmt.name, child_types, multiplicity))
#
#     return info
#
#
# def walk_project(root_dir: str) -> Dict[str, ClassInfo]:
#     classes: Dict[str, ClassInfo] = {}
#
#     for dirpath, _, filenames in os.walk(root_dir):
#         for filename in filenames:
#             if not filename.endswith(".py"):
#                 continue
#             full_path = os.path.join(dirpath, filename)
#             rel_module = os.path.relpath(full_path, root_dir)
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
#                     classes[node.name] = process_class(node, module_name)
#
#     return classes
#
#
# # ---------------------------------------------------------------------------
# # Filtering: keep GeomBase itself and every class that transitively inherits
# # from it (downward through the inheritance tree only).
# # ---------------------------------------------------------------------------
#
# def find_geombase_subclasses(classes: Dict[str, ClassInfo], root: str = "GeomBase") -> Set[str]:
#     kept: Set[str] = set()
#
#     # Build downward map: parent -> set of direct children
#     children_of: Dict[str, Set[str]] = {name: set() for name in classes}
#     for cls in classes.values():
#         for base in cls.bases:
#             if base in children_of:
#                 children_of[base].add(cls.name)
#
#     if root not in classes:
#         print(f"# Warning: '{root}' not found in parsed classes.", file=sys.stderr)
#         return set(classes.keys())
#
#     stack = [root]
#     while stack:
#         name = stack.pop()
#         if name in kept:
#             continue
#         kept.add(name)
#         for child in children_of.get(name, []):
#             stack.append(child)
#
#     return kept
#
#
# # ---------------------------------------------------------------------------
# # Relationships
# # ---------------------------------------------------------------------------
#
# def build_relationships(
#     classes: Dict[str, ClassInfo],
#     kept: Set[str],
# ) -> Tuple[
#     Set[Tuple[str, str]],
#     List[Tuple[str, str, str, Optional[Tuple[int, Optional[int]]], bool]],
# ]:
#     """
#     Returns:
#       inheritance  : set of (base, derived) where both are in `kept`
#       compositions : list of (parent, child_type, part_name, multiplicity, is_dynamic)
#                      is_dynamic=True when the @Part has >1 possible return type
#     """
#     inheritance: Set[Tuple[str, str]] = set()
#     compositions: List[Tuple[str, str, str, Optional[Tuple[int, Optional[int]]], bool]] = []
#
#     for cls in classes.values():
#         if cls.name not in kept:
#             continue
#         for base in cls.bases:
#             if base in kept:
#                 inheritance.add((base, cls.name))
#
#     for cls in classes.values():
#         if cls.name not in kept:
#             continue
#         for part in cls.parts:
#             is_dynamic = len(part.child_types) > 1
#             for child_type in part.child_types:
#                 if child_type in kept:
#                     compositions.append(
#                         (cls.name, child_type, part.name, part.multiplicity, is_dynamic)
#                     )
#
#     return inheritance, compositions
#
#
# # ---------------------------------------------------------------------------
# # PlantUML generation
# # ---------------------------------------------------------------------------
#
# def multiplicity_label(mult: Optional[Tuple[int, Optional[int]]]) -> str:
#     if mult is None:
#         return "1"
#     lo, hi = mult
#     if hi is None:
#         return f"{lo}..*"
#     if lo == hi:
#         return str(lo)
#     return f"{lo}..{hi}"
#
#
# def escape_puml(s: str) -> str:
#     return s
#
#
# def generate_plantuml(classes: Dict[str, ClassInfo], root: str = "GeomBase") -> str:
#     kept = find_geombase_subclasses(classes, root)
#     inheritance, compositions = build_relationships(classes, kept)
#
#     lines: List[str] = []
#     lines.append("@startuml")
#     lines.append("hide empty members")
#     lines.append("skinparam classAttributeIconSize 0")
#     lines.append("")
#
#     # Emit only kept classes
#     for cls in sorted((c for c in classes.values() if c.name in kept), key=lambda c: c.name):
#         members: List[str] = []
#
#         if cls.inputs:
#             for name, typ in cls.inputs:
#                 t = f": {typ}" if typ else ""
#                 members.append(f"  +{name}{t} <<Input>>")
#
#         if cls.attributes:
#             members.append("  .. Attributes ..")
#             for name in cls.attributes:
#                 members.append(f"  +{name}() <<Attribute>>")
#
#         if cls.parts:
#             members.append("  .. Parts ..")
#             for part in cls.parts:
#                 # Show dynamic options as TypeA|TypeB in the member listing
#                 type_label = "|".join(part.child_types) if part.child_types else ""
#                 suffix = f": {type_label}" if type_label else ""
#                 members.append(f"  +{part.name}(){suffix} <<Part>>")
#
#         if members:
#             lines.append(f"class {escape_puml(cls.name)} {{")
#             lines.extend(members)
#             lines.append("}")
#         else:
#             lines.append(f"class {escape_puml(cls.name)}")
#
#     lines.append("")
#
#     # Inheritance arrows
#     for base, derived in sorted(inheritance):
#         lines.append(f"{escape_puml(base)} <|-- {escape_puml(derived)}")
#
#     lines.append("")
#
#     # Composition arrows.
#     # Static parts  : filled diamond  *--  "mult"  Child : partName
#     # Dynamic parts : dashed arrow    ..>  "mult"  Child : partName  (one arrow per possible type)
#     seen_pairs: Set[Tuple[str, str, str]] = set()
#     for parent, child, part_name, mult, is_dynamic in sorted(
#         compositions, key=lambda x: (x[0], x[1], x[2])
#     ):
#         pair = (parent, child, part_name)
#         if pair in seen_pairs:
#             continue
#         seen_pairs.add(pair)
#
#         mult_str = multiplicity_label(mult)
#         if is_dynamic:
#             # Dashed arrow signals "one of these options is selected at runtime"
#             lines.append(
#                 f'{escape_puml(parent)} ..> "{mult_str}" {escape_puml(child)} : {part_name} (dynamic)'
#             )
#         else:
#             lines.append(
#                 f'{escape_puml(parent)} *-- "{mult_str}" {escape_puml(child)} : {part_name}'
#             )
#
#     lines.append("")
#     lines.append("@enduml")
#     return "\n".join(lines)
#
#
# # ---------------------------------------------------------------------------
# # Kroki POST rendering
# # ---------------------------------------------------------------------------
#
# def render_via_kroki(
#     puml: str,
#     output_path: str,
#     kroki_url: str = "https://kroki.io",
#     fmt: str = "png",
# ) -> None:
#     """
#     POST the PlantUML source to Kroki and write the response to output_path.
#
#     Kroki's POST endpoint:
#         POST {kroki_url}/plantuml/{fmt}
#         Content-Type: text/plain
#         Body: raw PlantUML source
#
#     This avoids the URI length limit entirely — there is no size ceiling on
#     POST bodies (unlike GET-based URI encoding which hits 4096–8192 chars).
#     """
#     import urllib.request
#     import urllib.error
#
#     endpoint = f"{kroki_url.rstrip('/')}/plantuml/{fmt}"
#     payload = puml.encode("utf-8")
#
#     req = urllib.request.Request(
#         endpoint,
#         data=payload,
#         method="POST",
#         headers={
#             "Content-Type": "text/plain",
#             "Accept": f"image/{fmt}",
#         },
#     )
#
#     print(f"POSTing {len(payload)} bytes to {endpoint} ...", file=sys.stderr)
#     try:
#         with urllib.request.urlopen(req) as resp:
#             image_data = resp.read()
#     except urllib.error.HTTPError as e:
#         body = e.read().decode("utf-8", errors="replace")
#         print(f"Kroki error {e.code}: {body}", file=sys.stderr)
#         raise
#
#     with open(output_path, "wb") as f:
#         f.write(image_data)
#
#     print(f"Saved {len(image_data)} bytes → {output_path}", file=sys.stderr)
#
#
# # ---------------------------------------------------------------------------
# # Entry point
# # ---------------------------------------------------------------------------
#
# USAGE = """\
# Usage:
#   {prog} <project_root_dir> [RootClass=GeomBase]
#          Print PlantUML source to stdout.
#
#   {prog} <project_root_dir> [RootClass=GeomBase] --kroki [output.png]
#          POST to Kroki and save the rendered image (default: diagram.png).
#
#   {prog} <project_root_dir> [RootClass=GeomBase] --kroki [output.svg] --fmt svg
#          Render as SVG instead of PNG.
#
#   {prog} <project_root_dir> [RootClass=GeomBase] --kroki [output.png] --kroki-url http://localhost:8000
#          Use a self-hosted Kroki instance.
# """
#
#
# def main(argv: List[str]) -> int:
#     import argparse
#
#     parser = argparse.ArgumentParser(
#         prog=os.path.basename(argv[0]),
#         formatter_class=argparse.RawDescriptionHelpFormatter,
#         description=USAGE,
#     )
#     parser.add_argument("project_root", help="Root directory of the Python project")
#     parser.add_argument("root_class", nargs="?", default="GeomBase",
#                         help="Base class to filter by (default: GeomBase)")
#     parser.add_argument("--kroki", nargs="?", const="diagram.png", metavar="OUTPUT",
#                         help="POST to Kroki and save the image (default filename: diagram.png)")
#     parser.add_argument("--fmt", default="png", choices=["png", "svg", "pdf"],
#                         help="Output format when using --kroki (default: png)")
#     parser.add_argument("--kroki-url", default="https://kroki.io",
#                         help="Kroki server URL (default: https://kroki.io)")
#
#     args = parser.parse_args(argv[1:])
#
#     root_dir = os.path.abspath(args.project_root)
#     if not os.path.isdir(root_dir):
#         print(f"Error: {root_dir} is not a directory", file=sys.stderr)
#         return 1
#
#     classes = walk_project(root_dir)
#     puml = generate_plantuml(classes, root=args.root_class)
#
#     if args.kroki is not None:
#         # Ensure the output extension matches the requested format
#         output = args.kroki
#         if not output.endswith(f".{args.fmt}"):
#             output = os.path.splitext(output)[0] + f".{args.fmt}"
#         render_via_kroki(puml, output, kroki_url=args.kroki_url, fmt=args.fmt)
#     else:
#         print(puml)
#
#     return 0
#
#
# if __name__ == "__main__":
#     raise SystemExit(main(sys.argv))