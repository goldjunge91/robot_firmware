# MCP Tools Reference

## Sequential Thinking Tools

The `sequentialthinking_tools` MCP server provides structured problem-solving through sequential thought processes with tool recommendations.

### When to Use

- Breaking down complex implementation tasks
- Planning multi-step solutions
- Analyzing codebases systematically
- Getting tool recommendations for each step
- Maintaining context across problem-solving steps

### Key Features

- **Thought Tracking**: Maintains numbered sequence of thoughts
- **Tool Recommendations**: Suggests appropriate tools with confidence scores (0-1)
- **Workflow Management**: Tracks previous steps, current step, remaining steps
- **Branching Support**: Can revise previous thoughts or branch into alternatives
- **Context Preservation**: Keeps full thought history

### Required Parameters

```json
{
  "available_mcp_tools": ["fileSearch", "grepSearch", "readFile", "strReplace", "executeBash"],
  "thought": "Current thinking step with analysis",
  "thought_number": 1,
  "total_thoughts": 5,
  "next_thought_needed": true,
  "previous_steps": [],
  "remaining_steps": ["Step 2", "Step 3"],
  "current_step": {
    "step_description": "What needs to be done",
    "recommended_tools": [
      {
        "tool_name": "fileSearch",
        "confidence": 0.9,
        "rationale": "Why this tool is recommended",
        "priority": 1,
        "alternatives": ["grepSearch"]
      }
    ],
    "expected_outcome": "What to expect from this step"
  }
}
```

### Important Notes

- `recommended_tools` is REQUIRED even when finishing (use empty array or "none" tool)
- Each tool recommendation needs: `tool_name`, `confidence`, `rationale`, `priority`
- `alternatives` field is optional but helpful
- Set `next_thought_needed: false` only when truly complete
- `total_thoughts` can be adjusted as understanding deepens

### Example Usage Pattern

1. Start with initial analysis and estimate total thoughts
2. Each thought builds on previous insights
3. Tool recommendations guide next actions
4. Adjust `total_thoughts` if needed
5. Complete when solution is clear

### Common Pitfalls

- Forgetting `recommended_tools` array in final step
- Missing required fields in tool recommendations
- Not providing `alternatives` array (even if empty)
- Setting `next_thought_needed: false` too early

