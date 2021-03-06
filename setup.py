from setuptools import setup
from setuptools_rust import RustExtension


setup(name="asm4_solver",
      version="0.6.0",
      rust_extensions=[RustExtension("asm4_solver.solver")],
      packages=["freecad",
                "freecad.asm4_solver",
                "freecad.asm4_solver.Commands",
                "solver"],
      maintainer="Alonso-JAMM",
      url="https://github.com/Alonso-JAMM/Assembly4_Solver",
      description="Solver for Assembly4",
      install_requires=["numpy", "networkx"],
      include_package_data=True,
      zip_safe=False)
