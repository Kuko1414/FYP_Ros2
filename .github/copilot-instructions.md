AI Agent Core System Instructions
As an AI coding assistant, you MUST STRICTLY adhere to the following rules before generating code, analyzing files, or executing tool calls:

1. When starting a new session or executing any workspace-wide file searches, ALWAYS read following MarkDown file to understand the system's architecture, future work and current state:
 - 'ARCHITECTURE.md' : This file contains the overall design of the system, including the goals, the ROS2 topics and nodes, and the workspace structure. It is critical to understand this file before making any changes to ensure that your code aligns with the overall design and goals of the system. Always refer to this file when you are unsure about how to implement a certain feature or when you need to understand how different components of the system interact with each other.
 - 'PROCESS.md'  ：This file contains the step-by-step process for achieving the goals outlined in 'ARCHITECTURE.md'. It provides detailed instructions on how to implement each feature and how to progress through the different stages of work. Always refer to this file when you are working on a specific task or when you need guidance on how to proceed with your work.
 - 'MEMORY.md'  ： This file contains a record of all the changes made to the workspace content. It is important to keep track of the changes made to the system to understand its evolution and to ensure that all changes are documented properly. Always refer to this file when you want to understand the history of changes made to the system or when you need to document a new change that you have made.

These files contain critical context that will inform your actions. Blindly crawing the entire workspace, is prohibited.

2. Communication Protocol
Language: ALWAYS repsond to me in Simplified Chinese (简体中文) unless strictly necessary for technical logs/code.

Clarity: Before explaining any code, provide a signle-sentence summary of the core logic.

3. Ask if unsure: When encountering ambiguity or design decisions, use AskUserQuestion to confirm with you rather than making unauthorized decisions.

4. Minimal changes: Only change that is absolutely necessary. No unnecessary 'hitchhiking' refactoring, and no overcomplicating things.

5. After generating code, ALWAYS provide a concise summary of the changes made in 'MEMORY.md' to keep track of the workspace's evolution. But to hole it simple, ONLY update the 'MEMORY.md' when I ask you to, and make sure to follow the format in 'MEMORY.md' for consistency. Each update should be in 100 words or less, and should clearly state what was changed, why it was changed, and how it fits into the overall design.

