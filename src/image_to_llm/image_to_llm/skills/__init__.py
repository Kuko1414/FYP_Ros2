"""
Skill 加载器：从 YAML 文件加载 Skill 配置。

用法：
    from image_to_llm.skills import load_skill, list_skills
    skill = load_skill("default")
    print(skill.system_prompt)
"""
import os
import yaml
from dataclasses import dataclass, field
from typing import List


@dataclass
class Skill:
    """一个 Skill 的完整配置。"""
    name: str
    description: str
    system_prompt: str
    model: str = ""
    required_tools: List[str] = field(default_factory=list)
    max_turns: int = 1


# skills/ 目录的绝对路径
_SKILLS_DIR = os.path.dirname(os.path.abspath(__file__))


def load_skill(skill_name: str) -> Skill:
    """从 skills/ 目录加载指定名称的 Skill YAML 文件。

    Args:
        skill_name: Skill 名称（不含 .yaml 后缀），如 "default"、"navigator"

    Returns:
        Skill 对象

    Raises:
        FileNotFoundError: 如果指定的 Skill YAML 文件不存在
    """
    yaml_path = os.path.join(_SKILLS_DIR, f"{skill_name}.yaml")

    if not os.path.exists(yaml_path):
        available = list_skills()
        raise FileNotFoundError(
            f"Skill '{skill_name}' not found at {yaml_path}. "
            f"Available skills: {available}"
        )

    with open(yaml_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    return Skill(
        name=data.get('name', skill_name),
        description=data.get('description', ''),
        system_prompt=data.get('system_prompt', ''),
        model=data.get('model', ''),
        required_tools=data.get('required_tools', []),
        max_turns=data.get('max_turns', 1),
    )


def list_skills() -> List[str]:
    """列出所有可用的 Skill 名称。"""
    return [
        f.replace('.yaml', '')
        for f in os.listdir(_SKILLS_DIR)
        if f.endswith('.yaml')
    ]
