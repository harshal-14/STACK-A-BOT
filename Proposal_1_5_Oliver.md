# AI Agents for Data to Ontology.

Build a team of AI agents whose task is to build an ontology when given access to a number of organisational data sources.

The value proposition is to enable stakeholders (managers, business analysts and other ai agents) to 
1. quickly understand the data landscape of an organisation
2. to have broad insight over all data
3. to know where to begin when querying a specific state or process.

This is a common problem in knowledge driven industries, where acquisiton and interrogation of new knowledge is key to wrealth creation. For example, pharmaceuticals and healthcare. Ontology projects in such industries are main pillars of data strategies, but can take years to complete, or are abandoned before completion. 

Full automation may not be possible, or even desirable as subject matter experts are key to such projects and their knowledge of how the institution functions is rarely captured in data which can be fed to an LLM. However, it would be beneficial to have automated creation of drafts to be discussed and to make updates as required. Then SME's can focus on key issues as agents carry out the heavy lifting. 

The sources should be representative of organisations' data landscapes, including:
1. Relational databases (MySQL, SQL Server, Postgres etc)
2. No SQL (MongoDB, CouchDB, AWS DynamoDB, etc)
3. Graph (Neo4j, AWS Neptune, StarDog)
4. Structured Files, TSV/CSV/XLSX etc

Analysis of unstructured files, such as PDF, is excluded from this work. In the light of recent LLM advancements, extraction of an ontology and knowledge graph from unstructured data is the subject of much recent research, for example:

- [Liang & Gu, 2025](https://arxiv.org/pdf/2501.14300)
- [Ye et al, 2023](https://arxiv.org/pdf/2312.03022v2)
- [Neo4j](https://neo4j.com/labs/genai-ecosystem/llm-graph-builder/)
- [StarDog Voicebox](https://stardog.ai/)

## Proposed Input - No Catalog

The proposed input is access to the source systems. No catalog is necessary for this project. In a wider application of such agents a catalog would be advantageous, simply to locate the hundreds or even thousands of databases within a large enterprise.

In realistic deployments ai agents would also need to negotiate the permissions required to both view the schema's of those sources and to interrgoate samples of data to better understand relationships and quality.

However, this project is of limited time so intends to present the agent team direct access to each source.

## Proposed Output - No Knowledge Graph

The output is an ontology, not a knowledge graph. The difference is described in a below section. In brief, it does not import all data into a single location. Instead, it creates a catalog, the class schema for the knowledge graph. 

This is intentional, ETL of all data into one place would be complex and is already the objective of many warehouse and datalake projects over the past decade. Those systems could be used as the source from which the ontology is built.

## Output Constraint

To enable users of the ontology to query source systems effectively, the ontology must link concepts back to their origins, which is not a standard feature of ontologies. The proposed  solution achieves this by:

- Embedding basic source system identifiers (e.g., "Sakila", "MFlix") directly into the ontology using standard OWL annotations (eg dc:source, where a class can have multiple sources).
- Programmatically generating a separate, user-friendly mapping document that details the precise source table/collection and column/field for each ontology term.
- This provides analysts with both high-level context within the ontology and detailed, actionable lineage information in an accessible format.

## Proposed Method

In the first instance the agent team will query the sources as a non-deterministic team, see below section for discussion of deterministic vs non-deterministic multi agent systems. 

### Multi Agent System, Not CoPilot

The team is not a conversational co-pilot, although that approach is taken by [StarDog](https://www.stardog.com/blog/agents-need-democratized-data-too) in the only comparable system discovered.

Instead, the agents work entirely autonomously, simply reporting their proposed ontology. It would be possible to allow a sbject matter expert to discuss those findings with the team and ask them to accommodate changes.

### Data Examples

Each datasource will yield a different ontology, so annealing steps will be required to unify those individual ontologies. Such steps will also use a jupyter notebook for the team to iterate and ensure code validity. The final output ontology will be expressed in OWL (Web Ontology Language, the W3C standard).

Since tools already exist for mapping RDBMS (eg MySQL) to ontology, eg:
- [OnTop](https://ontop-vkg.org/guide/concepts.html) 
- [Rel2Graph](https://arxiv.org/pdf/2310.01080)
It may make sense to execute such a tool first, then present the output as a starting point for the team.

Research has yielded no such tools for mapping NoSQL to ontology. Furthermore, popular graph databases such as Neo4j and AWS Neptune are 'schema-optional' and may not have an ontology, the agent team would have to propose an ontology for them. 

### Agent Framework

A convenient framework for the non-deterministic approach is Autogen, from which AG2 has since been derived. Teams can be granted access to the data sources and jupyter notebooks as the main tool. Jupyter access is especially powerful as we need not anticipate all required tools in advance. The team simply code the tools they need. 

The team then use notebooks to both plan and execute queries on a database, with the objective of building an ontology. Given frontier LLM's the tagents are well trained in common query languages; SQL, DDL, SPARQL, Cypher, MQL, MapReduce Views, OWL, etc.

Jupyter notebooks is the most flexible tool for this task, but may not be the optimal. Review of successful notebooks may yield 'lessons learnt' for the future executions. It is possible that some sub tasks should be deterministic agent workflows, or simple tool calls.

### Agent Coding in Jupyter

Autogen included Jupyter code execution, see my example at:
- [Github](https://github.com/olimoz/AI_Teams_AutoGen/blob/main/JupyterNotebooksForAutoGen_Capability.ipynb)

In the above example the notebook is the agent conversation, it is the full context for the conversation, notebook cells, code cells and code results. Including all aspcects of the notebook in the context window gives long contexts, but agents follow the process well.

AG2 includes a simplified variant of the original Autogen Jupyter coder, safer but more boring. See introduction at:
- [AG2](https://docs.ag2.ai/0.8.2/docs/user-guide/advanced-concepts/code-execution/)

### Agent Team

Implement a specialized team consisting of:

- Analyst
    - Plans the extraction/generation process, analyzes schema and data samples, designs the ontology structure.
- Coder
    - Writes and executes Python code (SQL, RDFlib, pandas) within the notebook based on the Analyst's plans.
- Critic
    - Reviews plans, code, analysis, and outputs for quality, correctness, and adherence to objectives.
- NotebookAgent
    - Executes code provided by the Coder.
- GroupChatManager
    - Orchestrates agent interactions.

### Agent Team Task Description:

The task description given to the team will be as follows. Note, this is not a workflow strictly enforced by the framework. It is the task which the agents must complete as they see fit. We could simply give them the objective of creating an Ontology and allow them to 

1. Programmatically connect to and inspect the source system schema
2. Analyze schema clarity; if insufficient, perform targeted data sampling to infer semantics and relationships.
3. Define OWL classes, data properties, and object properties using RDFlib based on combined schema and data analysis.
4. Serialize the final ontology into a standard OWL file format (e.g., .owl or .ttl).
5. Provenance Mapping: embed high-level source system identifiers within the ontology using OWL annotations (dc:source) and generate a separate, detailed mapping document (e.g., CSV) linking ontology terms to specific source tables/columns/fields.

# Prior Art

## Agents Building Ontologies & Knowledge Graphs from Structured Data

StarDog is the only organisation which was found to publicly advertise progress in using AI agents to build an ontology and hence a knowledge graph. The advantage being their understanding of context and semantics. However, this work is not open source and not available for inspection.

- [Stardog](https://www.stardog.com/blog/agents-need-democratized-data-too/)

There has also been work into multi agents for knowledge graph curation in multimodal data, namely text and image. This is intended to assist Q&A over multi modal data, a different use case. However, agents 2 and 4, as described in section 4.4, create the knowledge graph and are relevant to our work:

- [ICLR 2025](https://openreview.net/pdf/414dca3113ee456a119df87d2817178589b024fe.pdf)

**Agent 2:**

> This agent creates detailed and hierarchical domain models from textual input. Key capabilities include
> (a) Detailed entity extraction
> (b) Multi-level relationship identification
> (c) Hierarchical structure creation
> (d) Attribute detailing
> (e) Ontology integration and cross-domain connection identification
> This agent is essential in our framework, creating a detailed domain model with multi-level relationships and attributes that enable the construction of a precise and semantically rich knowledge graph.

**Agent 4:**

> This agent creates fully connected, hierarchical Knowledge Graphs in Neo4j from populated domain models. Key capabilities include:
> (a) Converting domain models to graph structures
> (b) Generating Cypher queries for graph creation
> (c) Ensuring full connectivity and hierarchy
> (d) Performing data cleaning and normalization
> (e) Optimizing graph structure and implementing advanced graph algorithms

## Agents Working with Knowledge Graphs

There has been substantial work enabling agents to better read knowledge graphs, usually motivated by the need to reaplce vector databases with graph databases in retrieval augmented generation (RAG).

- [Zhao et al, 2024](https://arxiv.org/pdf/2410.11531)
- [StarDog](https://www.stardog.com/blog/enterprise-ai-requires-the-fusion-of-llm-and-knowledge-graph/)
- [AG2](https://docs.ag2.ai/latest/docs/blog/2024/12/06/FalkorDB-Structured/)
- [LlamaIndex](https://www.llamaindex.ai/blog/constructing-a-knowledge-graph-with-llamaindex-and-memgraph)

These developments may be relevant as the agents need to review and revise the ontologies they create. This revision process arises as the teams sequentially integrate new data sources and reflect changes to existing sources.

Notably, LlamaIndex publish a number of different ways in which an agent can query a knowledge graph and compare the effectiveness of models:
- [LlamaIndex](https://www.llamaindex.ai/blog/building-knowledge-graph-agents-with-llamaindex-workflows)

In each case we can greatly increase accuracy by insisting the agents engage in guardrails (ask only questions the graph can answer), retry and verificaiton steps.

LlamaIndex also publish details on the complexities of asking agents to propsoe knowledge graphs, including entity deduplication. 

- [Customizing property graph index in LlamaIndex, 2024-06-11](https://www.llamaindex.ai/blog/customizing-property-graph-index-in-llamaindex)

# Example Data Sources

The data sources should overlap or inter relate in some way, for example, all be about a single subject. This is in order that the ontology is contiguous, as opposed to being islands of unrelated subjects.

A simple start to allow construction of the agent team, would be to use the sample databases. Each of the major proviers has a movies database. A simple subject for knowledge graph creatino, but a good place to start.

- MySQL - Sakila, a RDBMS of a fictional movie rental business
- Neo4j - Movies
- MongoDB - MFlix
- IMDB TSV files (available from Kaggle)
- MovieLens 24M CSV files (available via Kaggle)

Movies may prove too simplisitic to test the agents against realistically complex environments. However, this simplicity may prove a boon during design and early testing of the ai agents.

# Terminology

## Deterministic Teams vs Non Deterministic

**Deterministic Frameworks (e.g., LangGraph, CrewAI):**

- Force agents to follow predefined flowcharts or graphs.
Interactions are scripted and predictable.
- Think: Following a strict recipe or assembly line.
- Application: Where tasks where the steps to completion are well understood or regulated, requiring precise execution, eg refunds policy.

**Non-Deterministic Frameworks (e.g., Autogen, AG2):**

- Allow agents to converse and collaborate freely towards an objective. Interactions are dynamic and emergent, often (but not always) guided by an orchestrator.
- Think: A brainstorming meeting or improv session.
- Application: For tasks where the steps to completion are not immediately clear, eg business analysis of performance.

## Catalog, Taxonomy, Ontology & Knowledge Graph

**Data Catalog**
A Data Catalog is the inventory system for finding and understanding data assets. It may use a taxonomy for organization, but is not required to.

**Taxonomy:**
Organises information into a hierarchy (tree structure) based on "is a type of" relationships. Focuses on categories & subcategories. Taxonomies are often available 'off the shelf' and then adapted to each organisation's use.

**Ontology:**
Defines the types of things, their properties, and the types of relationships between them. Focuses on rules & schema of how things connect.

**Knowledge Graph:**
Represents specific entities (real things/data points) and their actual connections based on the ontology's rules. Focuses on network of linked facts, complete data.
