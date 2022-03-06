## Test of embedded mermaid diagrams in GitHub README.md
* https://github.com/mermaid-js/mermaid
* https://mermaid-js.github.io/mermaid/#/README
* https://mermaid-js.github.io/mermaid/#/Tutorials
* https://www.kubernetes.dev/blog/2021/12/01/improve-your-documentation-with-mermaid.js-diagrams/
* https://mermaid-js.github.io/mermaid-live-editor/
* https://github.blog/2022-02-14-include-diagrams-markdown-files-mermaid/

```mermaid
stateDiagram-v2
    [*] --> Init
    Init --> Read_A/D
    Read_A/D --> Joystick
    Joystick --> Read_A/D
    Read_A/D --> Transform
    Transform--> Map
    Map      --> Twistify
    Twistify --> Publish
    Publish  --> Read_A/D
```

```mermaid
flowchart TD
    Init([Init Param]) --> Read_A/D
    Joystick  -.-> |0-3.3V|Read_A/D
    Read_A/D  -.-> Joystick
    Read_A/D{{Read A/D}} --> |0-32000|Transform
    Transform --> |-100..+100|Map
    Map       --> |-100..+100|Twistify
    Twistify  --> |m/s + rad/s|Publish
    Publish   --> |Loop|Read_A/D
    click Init callback "Tooltip for a callback"
```

```mermaid
sequenceDiagram
    participant Alice
    participant John

    rect rgb(191, 223, 255)
    note right of Alice: Alice calls John.
    Alice->>+John: Hello John, how are you?
    rect rgb(200, 150, 255)
    Alice->>+John: John, can you hear me?
    John-->>-Alice: Hi Alice, I can hear you!
    end
    John-->>-Alice: I feel great!
    end
    Alice ->>+ John: Did you want to go to the game tonight?
    John -->>- Alice: Yeah! See you there.
```

```mermaid
sequenceDiagram
    autonumber
    Alice->>John: Hello John, how are you?
    loop Healthcheck
        John->>John: Fight against hypochondria
    end
    Note right of John: Rational thoughts!
    John-->>Alice: Great!
    John->>Bob: How about you?
    Bob-->>John: Jolly good!
```

```mermaid
sequenceDiagram
    actor Alice
    actor Bob
    Alice->>Bob: Hi Bob
    Bob->>Alice: Hi Alice
```

```mermaid
graph LR;
 client([client])-. Ingress-managed <br> load balancer .->ingress[Ingress];
 ingress-->|routing rule|service[Service];
 subgraph cluster
 ingress;
 service-->pod1[Pod];
 service-->pod2[Pod];
 end
 classDef plain fill:#ddd,stroke:#fff,stroke-width:4px,color:#000;
 classDef k8s fill:#326ce5,stroke:#fff,stroke-width:4px,color:#fff;
 classDef cluster fill:#fff,stroke:#bbb,stroke-width:2px,color:#326ce5;
 class ingress,service,pod1,pod2 k8s;
 class client plain;
 class cluster cluster;
 ```
