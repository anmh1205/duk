# Architecture Decision Records

Architecture Decision Records (ADRs) preserve durable choices that affect multiple DUK subsystems, interfaces, build practices or future engineering work. An ADR explains the context and trade-off that led to a decision; it does not duplicate the current API reference or replace the source code.

## When to record a decision

Create an ADR for changes such as the ownership of mathematical models, canonical physical-parameter representation, ROS interface boundaries, deployment architecture or long-lived data formats. Routine implementation details and short-term work plans belong in code reviews or issue discussions, not ADRs.

## Record format

Use a stable numbered filename such as `0001-brief-topic.md`, with:

- **Title and status** — proposed, accepted, superseded or deprecated.
- **Context** — relevant constraints, existing system behavior and the problem being solved.
- **Decision** — the architectural rule that applies.
- **Consequences** — resulting dependencies, migration requirements and trade-offs.
- **References** — affected source paths, related decisions or applicable external documentation.

A superseded decision remains available with a link to its replacement; avoid silently rewriting history. Canonical implementation details stay in the owning technical documents.
