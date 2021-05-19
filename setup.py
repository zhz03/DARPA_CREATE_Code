import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="sensing-planning-framework",
    version="0.0.1",
    author="Zhaoliang Zheng",
    author_email="aviros@tamu.edu",
    description="Sensing Planning Framework",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/CREATE-knowledge-planning/Sensing_planning_framework",
    packages=setuptools.find_packages(),
    python_requires='>=3.7',
)